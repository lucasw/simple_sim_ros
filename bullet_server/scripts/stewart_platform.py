#!/usr/bin/env python
# Create a stewart platform with service calls into bullet_server

import math
import rospy
import tf

from bullet_server.msg import Body, Constraint, Heightfield, Impulse
from bullet_server.srv import *


class StewartPlatform:
    def __init__(self):
        rospy.wait_for_service('add_compound')
        self.add_compound = rospy.ServiceProxy('add_compound', AddCompound)
        add_compound_request = AddCompoundRequest()

        floor = -9.0
        radius = 0.5
        height = 1.0
        rot90 = tf.transformations.quaternion_from_euler(math.pi/2.0, 0, 0)
        thickness = 0.1
        # make the bottom cylinder plate
        bot_plate = Body()
        bot_plate.name = "bottom_plate"
        bot_plate.pose.orientation.x = rot90[0]
        bot_plate.pose.orientation.y = rot90[1]
        bot_plate.pose.orientation.z = rot90[2]
        bot_plate.pose.orientation.w = rot90[3]
        bot_plate.pose.position.z = floor + 0.1
        bot_plate.type = Body.CYLINDER
        bot_plate.scale.x = radius
        bot_plate.scale.y = thickness
        bot_plate.scale.z = radius
        add_compound_request.body.append(bot_plate)

        # make the top cylinder plate
        top_plate = Body()
        top_plate.name = "top_plate"
        top_plate.pose.orientation.x = rot90[0]
        top_plate.pose.orientation.y = rot90[1]
        top_plate.pose.orientation.z = rot90[2]
        top_plate.pose.orientation.w = rot90[3]
        top_plate.pose.position.z = floor + height
        top_plate.type = Body.CYLINDER
        top_plate.scale.x = radius
        top_plate.scale.y = thickness
        top_plate.scale.z = radius
        add_compound_request.body.append(top_plate)

        # make six actuator cylinder bottoms with TBD spacing in a circle

        # connect the cylinders to the bottom plate with p2p ball socket
        # joints.

        # make six actuator cylinder tops

        # connect the top cylinders with p2p joints to top plate

        # connect each top cylinder to paired bottom cylinder with slider constraint 

        try:
            add_compound_response = self.add_compound(add_compound_request)
            rospy.loginfo(add_compound_response)
        except rospy.service.ServiceException as e:
            rospy.logerr(e)

if __name__ == '__main__':
    rospy.init_node('stewart_platform')
    stewart_platform = StewartPlatform()
