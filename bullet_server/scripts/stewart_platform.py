#!/usr/bin/env python
# Create a stewart platform with service calls into bullet_server

import copy
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
        add_compound_request.remove = rospy.get_param('~remove', False)

        floor = 0.0
        radius = 1.0
        height = radius * 2
        rot90 = tf.transformations.quaternion_from_euler(math.pi/2.0, 0, 0)
        thickness = 0.1
        # make the bottom cylinder plate
        bot_plate = Body()
        bot_plate.name = "bottom_plate"
        # make the plate static
        bot_plate.mass = 0.0
        bot_plate.pose.orientation.x = rot90[0]
        bot_plate.pose.orientation.y = rot90[1]
        bot_plate.pose.orientation.z = rot90[2]
        bot_plate.pose.orientation.w = rot90[3]
        bot_plate.pose.position.z = floor + 0.2
        bot_plate.type = Body.CYLINDER
        bot_plate.scale.x = radius
        bot_plate.scale.y = thickness
        bot_plate.scale.z = radius
        add_compound_request.body.append(bot_plate)

        # make the top cylinder plate
        top_plate = Body()
        top_plate.name = "top_plate"
        top_plate.mass = 1.0
        top_plate.pose.orientation.x = rot90[0]
        top_plate.pose.orientation.y = rot90[1]
        top_plate.pose.orientation.z = rot90[2]
        top_plate.pose.orientation.w = rot90[3]
        top_plate.pose.position.z = floor + height
        top_plate.type = Body.CYLINDER
        top_plate.scale.x = radius
        top_plate.scale.y = thickness
        top_plate.scale.z = radius
        # add_compound_request.body.append(top_plate)

        # make six actuator cylinder bottoms with TBD spacing in a circle
        for i in range(6):
            bot_cylinder = Body()
            bot_cylinder.name = "bot_cylinder_" + str(i)
            bot_cylinder.mass = 0.3
            bot_cylinder.pose.orientation.x = rot90[0]
            bot_cylinder.pose.orientation.y = rot90[1]
            bot_cylinder.pose.orientation.z = rot90[2]
            bot_cylinder.pose.orientation.w = rot90[3]
            bot_cylinder.pose.position.x = 0.7 * radius * math.cos(i/6.0 * 2.0 * math.pi)
            bot_cylinder.pose.position.y = floor + 0.2 + height/6.0 * 0.5 + 0.2
            bot_cylinder.pose.position.z = 0.7 * radius * math.sin(i/6.0 * 2.0 * math.pi)
            bot_cylinder.type = Body.CYLINDER
            bot_cylinder.scale.x = thickness / 2.0
            bot_cylinder.scale.y = height / 8.0
            bot_cylinder.scale.z = thickness / 2.0
            add_compound_request.body.append(bot_cylinder)

            # connect the cylinders to the bottom plate with p2p ball socket
            # joints.
            constraint = Constraint()
            constraint.name = "bot_cylinder_p2p_" + str(i)
            constraint.body_a = "bottom_plate"
            constraint.body_b = bot_cylinder.name
            constraint.type = Constraint.POINT2POINT
            constraint.pivot_in_a.x = bot_cylinder.pose.position.x
            constraint.pivot_in_a.y = bot_cylinder.pose.position.y + 0.2
            constraint.pivot_in_a.z = bot_cylinder.pose.position.z
            constraint.pivot_in_b.x = 0
            constraint.pivot_in_b.y = bot_cylinder.scale.y + 0.05
            constraint.pivot_in_b.z = 0
            add_compound_request.constraint.append(constraint)

            # make six actuator cylinder tops
            top_cylinder = copy.deepcopy(bot_cylinder)
            top_cylinder.name = "top_cylinder_" + str(i)
            top_cylinder.pose.position.y = floor - height
            add_compound_request.body.append(top_cylinder)

            # connect each top cylinder to paired bottom cylinder with slider constraint 
            prismatic = Constraint()
            prismatic.name = "prismatic_" + str(i)
            prismatic.body_a = bot_cylinder.name
            prismatic.body_b = top_cylinder.name
            prismatic.type = Constraint.SLIDER
            prismatic.pivot_in_a.x = 0
            prismatic.pivot_in_a.y = 0
            prismatic.pivot_in_a.z = 0
            prismatic.pivot_in_b.x = 0
            prismatic.pivot_in_b.y = 0
            prismatic.pivot_in_b.z = 0
            prismatic.lower_lim = 0.5
            prismatic.upper_lim = 1.5
            add_compound_request.constraint.append(prismatic)

            # connect the top cylinders with p2p joints to top plate

        try:
            add_compound_response = self.add_compound(add_compound_request)
            rospy.loginfo(add_compound_response)
        except rospy.service.ServiceException as e:
            rospy.logerr(e)

if __name__ == '__main__':
    rospy.init_node('stewart_platform')
    stewart_platform = StewartPlatform()
