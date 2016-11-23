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

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *


class InteractiveMarkerSpawn:
    def __init__(self):
        self.server = InteractiveMarkerServer("body_spawn")
        self.im = InteractiveMarker()
        self.im.header.frame_id = "map"
        self.im.name = "body_spawner"
        self.im.description = "Spawn a new body"

        self.marker = Marker()
        # TODO(lucasw) make a drop-down that can change this
        self.marker.type = Marker.CUBE
        scale_x = 0.5
        self.marker.scale.x = scale_x
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

        self.resize_x = InteractiveMarkerControl()
        self.resize_x.interaction_mode = InteractiveMarkerControl.BUTTON
        self.resize_x.name = "resize_x"
        arrow = Marker()
        arrow.type = Marker.ARROW
        arrow.pose.position.x = scale_x * 0.5
        arrow.scale.x = 0.8
        arrow.scale.y = 0.3
        arrow.scale.z = 0.3
        arrow.color.r = 0.1
        arrow.color.g = 0.9
        arrow.color.b = 0.15
        arrow.color.a = 1.0
        self.resize_x.markers.append(arrow)
        self.im.controls.append(self.resize_x)

        self.server.insert(self.im, self.process_feedback)
        self.server.applyChanges()

    def process_feedback(self, feedback):
        # print feedback
        if feedback.control_name == "resize_x":
            if feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
                scale_x = self.im.controls[0].markers[0].scale.x * 1.2
                # TODO(lucasw) make dict to get rid of hardcoding
                self.im.controls[0].markers[0].scale.x = scale_x
                self.im.controls[1].markers[0].pose.position.x = scale_x * 0.5
                # print "arrow button ", scale_x
                self.server.insert(self.im, self.process_feedback)
                self.server.applyChanges()

if __name__ == '__main__':
    rospy.init_node('imarker_spawn')
    imarker_spawn = InteractiveMarkerSpawn()
    rospy.spin()

