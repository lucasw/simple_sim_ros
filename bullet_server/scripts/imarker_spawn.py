#!/usr/bin/env python
# Lucas Walter
# November 2017

# Use interactive markers to create objects of supported shapes,
# size them, give them physical properties and then spawn them in
# the physics server.
# Need to be able to click on an object and get the marker for it-
# this may require more than just a simple interactive marker.
# Later being able to create joints between bodies would be nice.

import rospy

from interactive_markers.interactive_marker_server import *
from visualization_msg.msg import *


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
        self.marker.scale.x = 0.5
        self.marker.scale.y = 0.5
        self.marker.scale.z = 0.5
        self.marker.color.r = 0.5
        self.marker.color.g = 0.5
        self.marker.color.b = 0.5
        self.marker.color.a = 0.5

        self.marker_control = InteractiveMarkerControl()
        self.marker_control.always_visble = True
        self.marker_control.markers.append(self.marker)
        self.im.controls.append(self.marker_control)

if __name__ == '__main__':
    rospy.init_node('imarker_spawn')
    imarker_spawn = InteractiveMarkerSpawn()
    rospy.spin()

