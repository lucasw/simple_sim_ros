#!/usr/bin/env python
# Copyright (c) 2016 Lucas Walter
# November 2016

# Use interactive markers to create objects of supported shapes,
# size them, give them physical properties and then spawn them in
# the physics server.
# Need to be able to click on an object and get the marker for it-
# this may require more than just a simple interactive marker.
# Later being able to create joints between bodies would be nice.

# TODO(lucasw) this is taking up a lot of cpu

import copy
import numpy
import rospy
import tf
import tf2_ros

from bullet_server.msg import Body, Face, Link, Material, Node, SoftBody, SoftConfig, Tetra
from bullet_server.srv import *
from geometry_msgs.msg import Point, PointStamped, TransformStamped, Twist
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *


class InteractiveMarkerSpawn:
    def __init__(self):
        # self.br = tf2_ros.TransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.wait_for_service('add_compound')
        self.add_compound = rospy.ServiceProxy('add_compound', AddCompound)

        self.spawn_frame = rospy.get_param("~frame", "spawn_frame")
        self.ts = TransformStamped()
        self.ts.header.frame_id = "map"
        self.ts.child_frame_id = self.spawn_frame
        self.ts.transform.rotation.w = 1.0
        self.timer = rospy.Timer(rospy.Duration(0.1), self.update)

        self.count = 0
        self.server = InteractiveMarkerServer("body_spawn")
        self.im = InteractiveMarker()
        self.im.header.frame_id = self.spawn_frame
        self.im.name = "body_spawner"
        self.im.description = "Spawn a new body"

        self.menu_handler = MenuHandler()

        # TODO(lucasw) click and drag resizing will be best served
        # by and interactive marker child that has a frame tf defined
        # by the parent.

        menu = InteractiveMarkerControl()
        menu.interaction_mode = InteractiveMarkerControl.MENU
        menu.description = "spawn"
        menu.name = "spawn_menu"

        box = Marker()
        box.header.frame_id = self.spawn_frame
        box.type = Marker.CUBE
        box.scale.x = 0.5
        box.scale.y = 0.5
        box.scale.z = 0.5
        box.color.r = 0.8
        box.color.g = 0.8
        box.color.b = 0.8
        box.color.a = 1.0
        box.frame_locked = True
        menu.markers.append(box)
        self.im.controls.append(menu)

        self.menu_handler.insert("spawn", callback=self.process_feedback)
        # TODO(lucasw) have a submenu that is updated with a list
        # of all tf frames, and selecting one will cause that tf to
        # be the parent of this marker.

        self.server.insert(self.im, self.process_feedback)
        self.server.applyChanges()
        # TODO(lucasw) what does this do?
        self.menu_handler.apply(self.server, self.im.name)

        # resize x
        self.resize_x_server = InteractiveMarkerServer("body_spawn/resize_x")
        self.resize_x_im = InteractiveMarker()
        self.resize_x_im.header.frame_id = self.spawn_frame
        self.resize_x_im.name = "body_spawner_resize_x"
        self.resize_x_im.description = "resize x of new body"

        self.resize_x = InteractiveMarkerControl()
        self.resize_x.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        self.resize_x.name = "resize_x"
        self.resize_x.orientation.w = 1
        self.resize_x.orientation.x = 1
        self.resize_x.orientation.y = 0
        self.resize_x.orientation.z = 0

        arrow = Marker()
        arrow.type = Marker.ARROW
        # arrow.pose.position.x = box.scale.x * 0.5
        arrow.scale.x = 0.8
        arrow.scale.y = 0.3
        arrow.scale.z = 0.3
        arrow.color.r = 1.0
        arrow.color.g = 0.2
        arrow.color.b = 0.15
        arrow.color.a = 1.0
        arrow.frame_locked = True
        self.resize_x.markers.append(arrow)

        self.resize_x_im.controls.append(self.resize_x)
        self.resize_x_server.insert(self.resize_x_im, self.process_resize_feedback)
        self.resize_x_server.applyChanges()

        # resize y
        self.resize_y_server = InteractiveMarkerServer("body_spawn/resize_y")
        self.resize_y_im = InteractiveMarker()
        self.resize_y_im.header.frame_id = self.spawn_frame
        self.resize_y_im.name = "body_spawner_resize_y"
        self.resize_y_im.description = "resize y of new body"

        self.resize_y = InteractiveMarkerControl()
        self.resize_y.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        self.resize_y.name = "resize_y"
        self.resize_y.orientation.w = 1
        self.resize_y.orientation.x = 0
        self.resize_y.orientation.y = 0
        self.resize_y.orientation.z = 1

        arrow_y = copy.deepcopy(arrow)
        arrow_y.color.r = 0.2
        arrow_y.color.g = 1.0
        arrow_y.color.b = 0.15
        arrow_y.pose.orientation = self.resize_y.orientation
        self.resize_y.markers.append(arrow_y)
        self.resize_y_im.controls.append(self.resize_y)
        self.resize_y_server.insert(self.resize_y_im, self.process_resize_feedback)
        self.resize_y_server.applyChanges()

        # resize y
        self.resize_z_server = InteractiveMarkerServer("body_spawn/resize_z")
        self.resize_z_im = InteractiveMarker()
        self.resize_z_im.header.frame_id = self.spawn_frame
        self.resize_z_im.name = "body_spawner_resize_z"
        self.resize_z_im.description = "resize z of new body"

        self.resize_z = InteractiveMarkerControl()
        self.resize_z.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        self.resize_z.name = "resize_z"
        self.resize_z.orientation.w = 1
        self.resize_z.orientation.x = 0
        self.resize_z.orientation.y = 1
        self.resize_z.orientation.z = 0

        arrow_z = copy.deepcopy(arrow)
        arrow_z.color.r = 0.2
        arrow_z.color.g = 0.04
        arrow_z.color.b = 1.0
        arrow_z.pose.orientation = self.resize_z.orientation
        self.resize_z.markers.append(arrow_z)
        self.resize_z_im.controls.append(self.resize_z)
        self.resize_z_server.insert(self.resize_z_im, self.process_resize_feedback)
        self.resize_z_server.applyChanges()

        # add linear impulse
        self.linear_vel_server = InteractiveMarkerServer("body_spawn/linear_vel")
        self.linear_vel_im = InteractiveMarker()
        self.linear_vel_im.header.frame_id = self.spawn_frame
        self.linear_vel_im.name = "body_spawner_linear_vel"
        self.linear_vel_im.description = "add impulse to body"

        self.linear_vel_pt = [0, 0, 0, 0]
        self.linear_vel_control = InteractiveMarkerControl()
        self.linear_vel_control.interaction_mode = InteractiveMarkerControl.MOVE_3D
        self.linear_vel_control.name = "linear_vel"
        self.linear_vel_control.orientation.w = 1
        self.linear_vel_control.orientation.x = 0
        self.linear_vel_control.orientation.y = 1
        self.linear_vel_control.orientation.z = 0

        box = Marker()
        box.header.frame_id = self.spawn_frame
        box.type = Marker.CUBE
        box.scale.x = 0.1
        box.scale.y = 0.1
        box.scale.z = 0.1
        box.color.r = 0.8
        box.color.g = 0.8
        box.color.b = 0.2
        box.color.a = 1.0
        box.frame_locked = True
        self.linear_vel_control.markers.append(box)

        self.linear_vel_im.controls.append(self.linear_vel_control)
        self.linear_vel_server.insert(self.linear_vel_im, self.process_vel_feedback)
        self.linear_vel_server.applyChanges()

        self.marker_pub = rospy.Publisher("/body_spawn/misc_markers", Marker, queue_size=1)
        self.linear_vel_marker = Marker()
        self.linear_vel_marker.header.frame_id = self.spawn_frame
        self.linear_vel_marker.type = Marker.LINE_LIST
        self.linear_vel_marker.scale.x = 0.07
        self.linear_vel_marker.scale.y = 0.07
        self.linear_vel_marker.scale.z = 0.07
        self.linear_vel_marker.color.r = 0.8
        self.linear_vel_marker.color.g = 0.8
        self.linear_vel_marker.color.b = 0.2
        self.linear_vel_marker.color.a = 1.0
        self.linear_vel_marker.frame_locked = True
        pt = Point()
        self.linear_vel_marker.points.append(pt)
        pt = Point()
        self.linear_vel_marker.points.append(pt)
        self.marker_pub.publish(self.linear_vel_marker)

    def process_vel_feedback(self, feedback):
        if feedback.control_name == "linear_vel":
            # need to transform this point into the map frame,
            # but subtract out the location of the spawn frame
            self.linear_vel_pt = [feedback.pose.position.x * 4.0,
                                  feedback.pose.position.y * 4.0,
                                  feedback.pose.position.z * 4.0,
                                  1.0]
            self.linear_vel_marker.points[1] = feedback.pose.position
            self.marker_pub.publish(self.linear_vel_marker)

    def process_resize_feedback(self, feedback):
        if feedback.control_name == "resize_x":
            # if feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
            scale_x = 2.0 * abs(feedback.pose.position.x)
            # TODO(lucasw) make dict to get rid of hardcoding
            self.im.controls[0].markers[0].scale.x = scale_x
            self.server.insert(self.im, self.process_feedback)
            self.server.applyChanges()
        if feedback.control_name == "resize_y":
            # if feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
            scale_y = 2.0 * abs(feedback.pose.position.y)
            # TODO(lucasw) make dict to get rid of hardcoding
            self.im.controls[0].markers[0].scale.y = scale_y
            self.server.insert(self.im, self.process_feedback)
            self.server.applyChanges()
        if feedback.control_name == "resize_z":
            # if feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
            scale_z = 2.0 * abs(feedback.pose.position.z)
            # TODO(lucasw) make dict to get rid of hardcoding
            self.im.controls[0].markers[0].scale.z = scale_z
            self.server.insert(self.im, self.process_feedback)
            self.server.applyChanges()

    def update(self, event):
        pass

    def process_feedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            # feedback.control_name == "spawn_menu":
            if feedback.menu_entry_id == 1:
                self.count += 1
                # rospy.loginfo(feedback)
                body = Body()
                body.name = "imarker_spawned_body_" + str(self.count)
                # TODO(lucasw)
                body.mass = 1.0
                try:
                    trans = self.tf_buffer.lookup_transform("map", self.spawn_frame,
                                                            rospy.Time())
                except (tf2_ros.LookupException,
                        tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException) as e:
                    rospy.logerr("tf2_ros exception")
                    rospy.logerr(e)
                    return
                body.pose.position.x = trans.transform.translation.x
                body.pose.position.y = trans.transform.translation.y
                body.pose.position.z = trans.transform.translation.z
                body.pose.orientation = trans.transform.rotation

                # TODO(lucasw) add twist linear with another interactive tf,
                # and twist angular with a second interactive tf?
                body.type = Body.BOX
                body.scale.x = self.im.controls[0].markers[0].scale.x / 2.0
                body.scale.y = self.im.controls[0].markers[0].scale.y / 2.0
                body.scale.z = self.im.controls[0].markers[0].scale.z / 2.0

                # can't get this to work
                # http://answers.ros.org/question/249433/tf2_ros-buffer-transform-pointstamped/
                if False:
                    feedback_pt = PointStamped()
                    feedback_pt.header = feedback.header
                    feedback_pt.point.x = self.linear_vel_pt[0]
                    feedback_pt.point.y = self.linear_vel_pt[1]
                    feedback_pt.point.z = self.linear_vel_pt[2]

                    self.tf_buffer.registration.print_me()
                    try:
                        pt_in_map = self.tf_buffer.transform(feedback_pt, "map",
                                                             rospy.Duration(2.0), PointStamped)
                    except tf2_ros.TypeException as e:
                        # rospy.logerr(e)
                        print e
                        return
                    print 'output', pt_in_map
                else:
                    quat = [trans.transform.rotation.x,
                            trans.transform.rotation.y,
                            trans.transform.rotation.z,
                            trans.transform.rotation.w]
                    mat = tf.transformations.quaternion_matrix(quat)
                    pt_in_map = numpy.dot(mat, self.linear_vel_pt)
                    body.twist.linear.x = pt_in_map[0]
                    body.twist.linear.y = pt_in_map[1]
                    body.twist.linear.z = pt_in_map[2]

                # rospy.loginfo(body.twist.linear)

                add_compound_request = AddCompoundRequest()
                add_compound_request.remove = False
                add_compound_request.body.append(body)

                try:
                    add_compound_response = self.add_compound(add_compound_request)
                    rospy.loginfo(add_compound_response)
                except rospy.service.ServiceException as e:
                    rospy.logerr(e)


if __name__ == '__main__':
    rospy.init_node('imarker_spawn')
    imarker_spawn = InteractiveMarkerSpawn()
    rospy.spin()
