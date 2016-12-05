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
        # self.just_resized = False
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
        arrow.pose.position.x = box.scale.x * 0.5
        arrow.scale.x = 0.8
        arrow.scale.y = 0.3
        arrow.scale.z = 0.3
        arrow.color.r = 0.1
        arrow.color.g = 0.9
        arrow.color.b = 0.15
        arrow.color.a = 1.0
        arrow.frame_locked = True
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
                self.im.controls[0].markers[0].scale.x = self.scale_x
                self.server.insert(self.im, self.process_feedback)
                self.server.applyChanges()
                # self.just_resized = True

    def update(self, event):
        pass

    def process_feedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            # feedback.control_name == "spawn_menu":
            if feedback.menu_entry_id == 1:
		self.count += 1
		rospy.loginfo(feedback)
		body = Body()
		body.name = "imarker_spawned_body_" + str(self.count)
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

