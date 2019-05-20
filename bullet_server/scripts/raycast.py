#!/usr/bin/env python
# Lucas Walter 2017
#
# Create a ground plane (maybe do this in a launch file.
# Create 1 or more objects to bounce a range sensor off of.
# Create an object to hang the range sensor off of
# (all of above down in imarker.launch)
#
# Create the range sensor with the tf frame of that object as frame_id.

import math
import rospy

from bullet_server.msg import Line
from bullet_server.srv import *


if __name__ == '__main__':
    rospy.init_node('raycast')
    rospy.wait_for_service('add_raycast')
    add_raycast = rospy.ServiceProxy('add_raycast', AddRaycast)
    add_raycast_request = AddRaycastRequest()
    add_raycast_request.frame_id = rospy.get_param('~frame', "foo")
    add_raycast_request.name = "laser_range"
    add_raycast_request.topic_name = "laser_range"

    num_pts = 30
    half_pts = num_pts * 0.5
    half_angle = 0.5
    for i in range(num_pts):
        angle = half_angle * (i - half_pts) / half_pts
        line = Line()
        line.start.x = 0.1 * math.cos(angle)
        line.start.y = 0.0
        line.start.z = 0.1 * math.sin(angle)
        line.end.x = 10.0 * math.cos(angle)
        line.end.y = 0.0
        line.end.z = 10.0 * math.sin(angle)

        add_raycast_request.lines.append(line)
    rospy.loginfo(add_raycast_request)
    try:
        add_raycast_response = add_raycast(add_raycast_request)
        rospy.loginfo(add_raycast_response)
    except rospy.service.ServiceException as e:
        rospy.logerr(e)
