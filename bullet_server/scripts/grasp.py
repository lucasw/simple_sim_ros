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

        if True:
            thickness = 0.08
            cyl_length = 0.5
            arm_upper = Body()
            # arm_upper = copy.deepcopy(arm_fore)
            arm_upper.name = "arm_upper"
            arm_upper.mass = 0.3
            arm_upper.pose.position.x = 0.0
            arm_upper.pose.position.y = 0.0
            rot90 = tf.transformations.quaternion_from_euler(math.pi/2.0, 0, 0)
            arm_upper.pose.position.z = arm_base_height - cyl_length / 2.0
            arm_upper.pose.orientation.x = rot90[0]
            arm_upper.pose.orientation.y = rot90[1]
            arm_upper.pose.orientation.z = rot90[2]
            arm_upper.pose.orientation.w = rot90[3]
            arm_upper.type = Body.CYLINDER
            arm_upper.scale.x = thickness / 2.0
            arm_upper.scale.y = cyl_length / 2.0
            arm_upper.scale.z = thickness / 2.0
            add_compound_request.body.append(arm_upper)

            # connect the cylinders to the bottom plate with p2p ball socket
            # joints.
            constraint = Constraint()
            constraint.name = "arm_upper_fixed"
            constraint.body_a = "arm_base"
            constraint.body_b = arm_upper.name
            constraint.type = Constraint.FIXED
            # Need to transform arm_fore.pose into bot_plate frame
            # to get these
            constraint.pivot_in_a.x = 0.0
            constraint.pivot_in_a.y = 0.0
            constraint.pivot_in_a.z = 0.0
            constraint.pivot_in_b.x = 0
            constraint.pivot_in_b.y = cyl_length / 2.0
            constraint.pivot_in_b.z = 0
            add_compound_request.constraint.append(constraint)

            arm_fore = Body()
            arm_fore.name = "arm_fore"
            arm_fore.mass = 0.3
            arm_fore.pose.orientation.x = rot90[0]
            arm_fore.pose.orientation.y = rot90[1]
            arm_fore.pose.orientation.z = rot90[2]
            arm_fore.pose.orientation.w = rot90[3]
            arm_fore.pose.position.x = 0.0
            arm_fore.pose.position.y = 0.0
            arm_fore.pose.position.z = arm_base_height - cyl_length
            arm_fore.type = Body.CYLINDER
            arm_fore.scale.x = thickness * 0.4
            arm_fore.scale.y = cyl_length / 2.0
            arm_fore.scale.z = thickness * 0.4
            add_compound_request.body.append(arm_fore)

            # connect each top cylinder to paired bottom cylinder with slider constraint
            prismatic = Constraint()
            prismatic.name = "prismatic_upper_fore"
            prismatic.body_a = arm_upper.name
            prismatic.body_b = arm_fore.name
            prismatic.type = Constraint.SLIDER
            prismatic.enable_pos_pub = True
            prismatic.enable_motor_sub = True
            prismatic.pivot_in_a.x = 0.0
            prismatic.pivot_in_a.y = -cyl_length / 4.0
            prismatic.pivot_in_a.z = 0.0
            prismatic.pivot_in_b.x = 0.0
            prismatic.pivot_in_b.y = 0.0  # cyl_length / 4.0
            prismatic.pivot_in_b.z = 0.0
            prismatic.lower_lin_lim = 0.0
            prismatic.upper_lin_lim = cyl_length
            # TODO(lucasw) is this an absolute angle or rate?
            prismatic.lower_ang_lim = -0.1
            prismatic.upper_ang_lim = 0.1
            prismatic.max_motor_impulse = 1000.0
            add_compound_request.constraint.append(prismatic)

        try:
            add_compound_response = self.add_compound(add_compound_request)
            rospy.loginfo(add_compound_response)
        except rospy.service.ServiceException as e:
            rospy.logerr(e)

if __name__ == '__main__':
    rospy.init_node('stewart_platform')
    grasp = Grasp()
