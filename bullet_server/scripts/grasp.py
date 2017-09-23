#!/usr/bin/env python
# Create a stewart platform with service calls into bullet_server

import copy
import math
import rospy
import tf

from bullet_server.msg import Body, Constraint, Heightfield, Impulse
from bullet_server.srv import *


class Grasp:
    def __init__(self):
        rospy.loginfo("waiting for server add_compound")
        rospy.wait_for_service('add_compound')
        self.add_compound = rospy.ServiceProxy('add_compound', AddCompound)
        rospy.loginfo("connected to add_compound service")
        add_compound_request = AddCompoundRequest()
        add_compound_request.remove = rospy.get_param('~remove', False)

        # make a table
        table_height = 0.0
        rot90 = tf.transformations.quaternion_from_euler(math.pi/2.0, 0, 0)
        table_thickness = 0.1
        table = Body()
        table.name = "table"
        # make the floor static
        table.mass = 0.0
        table.pose.orientation.x = rot90[0]
        table.pose.orientation.y = rot90[1]
        table.pose.orientation.z = rot90[2]
        table.pose.orientation.w = rot90[3]
        table.pose.position.z = table_height - table_thickness / 2
        table.type = Body.CYLINDER
        table.scale.x = 1.0
        table.scale.y = table_thickness
        table.scale.z = 1.0
        add_compound_request.body.append(table)

        # make a sphere to grasp
        ball_radius = 0.04
        ball = Body()
        ball.name = "ball"
        ball.mass = 0.5
        ball.pose.orientation.x = rot90[0]
        ball.pose.orientation.y = rot90[1]
        ball.pose.orientation.z = rot90[2]
        ball.pose.orientation.w = rot90[3]
        ball.pose.position.z = table_height + ball_radius + 0.01
        ball.type = Body.SPHERE
        ball.scale.x = ball_radius
        ball.scale.y = ball_radius
        ball.scale.z = ball_radius
        add_compound_request.body.append(ball)

        # Platform arm is attached to
        arm_base_height = table_height + 1.0
        rot90 = tf.transformations.quaternion_from_euler(math.pi/2.0, 0, 0)
        arm_base_thickness = 0.02
        arm_base = Body()
        arm_base.name = "arm_base"
        # make the platform static
        arm_base.mass = 0.0
        arm_base.pose.orientation.x = rot90[0]
        arm_base.pose.orientation.y = rot90[1]
        arm_base.pose.orientation.z = rot90[2]
        arm_base.pose.orientation.w = rot90[3]
        arm_base.pose.position.z = arm_base_height
        arm_base.type = Body.CYLINDER
        arm_base.scale.x = 0.1
        arm_base.scale.y = arm_base_thickness
        arm_base.scale.z = 0.1
        add_compound_request.body.append(arm_base)

        try:
            add_compound_response = self.add_compound(add_compound_request)
            rospy.loginfo(add_compound_response)
        except rospy.service.ServiceException as e:
            rospy.logerr(e)

if __name__ == '__main__':
    rospy.init_node('stewart_platform')
    grasp = Grasp()
