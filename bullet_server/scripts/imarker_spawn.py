#!/usr/bin/env python
# Copyright (c) 2016 Lucas Walter
# November 2016

# Use interactive markers to create objects of supported shapes,
# size them, give them physical properties and then spawn them in
# the physics server.
# Need to be able to click on an object and get the marker for it-
# this may require more than just a simple interactive marker.
# Later being able to create joints between bodies would be nice.

import rospy
import tf
import tf2_ros

from bullet_server.msg import Body, Face, Link, Material, Node, SoftBody, SoftConfig, Tetra
from bullet_server.srv import *
from geometry_msgs.msg import TransformStamped
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *


class InteractiveMarkerSpawn:
    def __init__(self):
        self.br = tf2_ros.TransformBroadcaster()

        rospy.wait_for_service('add_compound')
        self.add_compound = rospy.ServiceProxy('add_compound', AddCompound)

        # Ground
        if True:
            add_compound_request = AddCompoundRequest()
            add_compound_request.remove = False
            # make the top cylinder plate
            ground = Body()
            ground.name = "ground"
            ground.mass = 0.0
            rot90 = tf.transformations.quaternion_from_euler(0, 0, 0)
            radius = 50
            thickness = 1.0
            ground.pose.orientation.x = rot90[0]
            ground.pose.orientation.y = rot90[1]
            ground.pose.orientation.z = rot90[2]
            ground.pose.orientation.w = rot90[3]
            ground.pose.position.z = -thickness
            ground.type = Body.BOX
            ground.scale.x = radius
            ground.scale.y = radius
            ground.scale.z = thickness
            add_compound_request.body.append(ground)

	    try:
		add_compound_response = self.add_compound(add_compound_request)
		rospy.loginfo(add_compound_response)
	    except rospy.service.ServiceException as e:
		rospy.logerr(e)

        self.spawn_frame = rospy.get_param("~frame", "spawn_frame")
        self.ts = TransformStamped()
        self.ts.header.frame_id = "map"
        self.ts.child_frame_id = self.spawn_frame
        self.ts.transform.rotation.w = 1.0
        # self.just_resized = False
        self.first_resize = True
        self.timer = rospy.Timer(rospy.Duration(0.1), self.update)

	self.count = 0
        self.server = InteractiveMarkerServer("body_spawn")
        self.im = InteractiveMarker()
        self.im.header.frame_id = "map"
        self.im.name = "body_spawner"
        self.im.description = "Spawn a new body"

        self.menu_handler = MenuHandler()

        self.marker = Marker()
        # TODO(lucasw) make a drop-down that can change this
        self.marker.type = Marker.CUBE
        self.scale_x = 0.5
        self.marker.scale.x = self.scale_x
        self.marker.scale.y = 0.5
        self.marker.scale.z = 0.5
        self.marker.color.r = 0.5
        self.marker.color.g = 0.5
        self.marker.color.b = 0.5
        self.marker.color.a = 1.0

        self.move_3d = InteractiveMarkerControl()
        self.move_3d.name = "body_spawn"
        self.move_3d.always_visible = True
        self.move_3d.interaction_mode = InteractiveMarkerControl.MOVE_3D
        self.move_3d.markers.append(self.marker)
        self.im.controls.append(self.move_3d)

        # TODO(lucasw) click and drag resizing will be best served
        # by and interactive marker child that has a frame tf defined
        # by the parent.

        menu = InteractiveMarkerControl()
        menu.interaction_mode = InteractiveMarkerControl.MENU
        menu.description = "Spawn"
        menu.name = "spawn_menu"
        self.im.controls.append(menu)

        self.menu_handler.insert("Spawn", callback=self.process_feedback)
        # TODO(lucasw) have a submenu that is updated with a list
        # of all tf frames, and selecting one will cause that tf to
        # be the parent of this marker.

        self.server.insert(self.im, self.process_feedback)
        self.server.applyChanges()
        # TODO(lucasw) what does this do?
        self.menu_handler.apply(self.server, self.im.name)

        #
        self.resize_x_server = InteractiveMarkerServer("body_spawn/resize_x")

        self.resize_x_im = InteractiveMarker()
        self.resize_x_im.header.frame_id = self.spawn_frame
        self.resize_x_im.name = "body_spawner_resize_x"
        self.resize_x_im.description = "resize x of new body"

        self.resize_x = InteractiveMarkerControl()
        self.resize_x.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        self.resize_x.name = "resize_x"

        arrow = Marker()
        arrow.type = Marker.ARROW
        arrow.pose.position.x = self.scale_x * 0.5
        arrow.scale.x = 0.8
        arrow.scale.y = 0.3
        arrow.scale.z = 0.3
        arrow.color.r = 0.1
        arrow.color.g = 0.9
        arrow.color.b = 0.15
        arrow.color.a = 1.0
        self.resize_x.markers.append(arrow)

        self.resize_x_im.controls.append(self.resize_x)
        self.resize_x_server.insert(self.resize_x_im, self.process_resize_x_feedback)
        self.resize_x_server.applyChanges()

    def process_resize_x_feedback(self, feedback):
        if feedback.control_name == "resize_x":
            # if feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
            print feedback
            if True:
                self.scale_x = 2.0 * abs(feedback.pose.position.x)
                # TODO(lucasw) make dict to get rid of hardcoding
                # the move_3d box marker
                # Have to set the marker to where it already is, otherwise
                # the initial values will overwrite it.
                # But only do it first time
                if self.first_resize:
                    self.first_resize = False
                    # if self.im.controls[0].markers[0].pose.position.x == 0 and
                    #         self.im.controls[0].markers[0].pose.position.y == 0
                    #         self.im.controls[0].markers[0].pose.position.z == 0:
                    self.im.controls[0].markers[0].pose.position.x = self.ts.transform.translation.x
                    self.im.controls[0].markers[0].pose.position.y = self.ts.transform.translation.y
                    self.im.controls[0].markers[0].pose.position.z = self.ts.transform.translation.z
                self.im.controls[0].markers[0].scale.x = self.scale_x
                self.server.insert(self.im, self.process_feedback)
                self.server.applyChanges()
                # self.just_resized = True

    def update(self, event):
        self.br.sendTransform(self.ts)
        # self.just_tf_updated = True
        # print feedback
        # if self.just_resized:
        #     self.just_resized = False
         
    def process_feedback(self, feedback):
        # TODO(lucasw) or use an rviz_interactive_marker to generate this tf?
        self.ts.header.stamp = feedback.header.stamp
        self.ts.transform.translation.x = feedback.pose.position.x
        self.ts.transform.translation.y = feedback.pose.position.y
        self.ts.transform.translation.z = feedback.pose.position.z
        self.ts.transform.rotation.x = feedback.pose.orientation.x
        self.ts.transform.rotation.y = feedback.pose.orientation.y
        self.ts.transform.rotation.z = feedback.pose.orientation.z
        self.ts.transform.rotation.w = feedback.pose.orientation.w

        self.first_resize = True
        if False:
            self.im.controls[0].markers[0].pose.position.x = 0
            self.im.controls[0].markers[0].pose.position.y = 0
            self.im.controls[0].markers[0].pose.position.z = 0
            self.server.insert(self.im, self.process_feedback)
            self.server.applyChanges()

        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            # feedback.control_name == "spawn_menu":
            if feedback.menu_entry_id == 1:
		self.count += 1
		rospy.loginfo(feedback)
		body = Body()
		body.name = "imarker_spawned_body_" + str(self.count)
		body.mass = 1.0
		body.pose.position.x = feedback.pose.position.x
		body.pose.position.y = feedback.pose.position.y
		body.pose.position.z = feedback.pose.position.z
		body.pose.orientation.x = feedback.pose.orientation.x
		body.pose.orientation.y = feedback.pose.orientation.y
		body.pose.orientation.z = feedback.pose.orientation.z
		body.pose.orientation.w = feedback.pose.orientation.w
		body.type = Body.BOX
		body.scale.x = self.im.controls[0].markers[0].scale.x / 2.0
		body.scale.y = self.im.controls[0].markers[0].scale.y / 2.0
		body.scale.z = self.im.controls[0].markers[0].scale.z / 2.0

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

