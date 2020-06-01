#!/usr/bin/env python
# Copyright Lucas Walter 2016

import copy
import math
import rospy
import tf

from bullet_server.msg import Anchor, Body, Constraint, Face, Link
from bullet_server.msg import Material, Node, SoftBody, SoftConfig, Tetra
from bullet_server.srv import *
from bullet_server.utility import *


class SoftVehicle:
    def __init__(self):
        rospy.wait_for_service('add_compound')
        self.add_compound = rospy.ServiceProxy('add_compound', AddCompound)
        add_compound_request = AddCompoundRequest()
        add_compound_request.remove = rospy.get_param('~remove', False)

        xs = rospy.get_param("~x", 0.0)
        ys = rospy.get_param("~y", 0.0)
        zs = rospy.get_param("~z", 1.0)

        # Ground
        if True:
            ground_radius = 50
            ground_thickness = 1.0
            ground_mass = 0.0
            ground = make_rigid_cylinder("ground", ground_mass,
                                         0, 0, -ground_thickness,
                                         ground_radius,
                                         ground_thickness,
                                         math.pi/2.0, 0, 0)
            add_compound_request.body.append(ground)

            # obstacles
            obstacle = make_rigid_cylinder("obstacle1", ground_mass,
                                           -10.0, 1.0, -ground_thickness + 0.4,
                                           3.0,
                                           ground_thickness,
                                           math.pi/2.0, 0, 0)
            add_compound_request.body.append(obstacle)
            obstacle = make_rigid_box("obstacle2", ground_mass,
                                      -15.0, 1.5, 0.2,
                                      3.0, 1.0, 0.5,
                                      0.1, 0.05, 0.01)
            add_compound_request.body.append(obstacle)

        chassis = make_rigid_box("chassis", 1.5, xs, ys, zs, 1.8, 0.5, 0.3)
        add_compound_request.body.append(chassis)

        motor_y = 0.8

        (motor, hinge, wheel) = make_wheel_assembly("front_left",
                                                    xs - 1.5, ys - motor_y, zs, flip=1.0)
        add_compound_request.body.append(motor)
        add_compound_request.constraint.append(hinge)
        add_compound_request.soft_body.append(wheel)

        (motor, hinge, wheel) = make_wheel_assembly("back_left",
                                                    xs + 1.5, ys - motor_y, zs, flip=1.0)
        add_compound_request.body.append(motor)
        add_compound_request.constraint.append(hinge)
        add_compound_request.soft_body.append(wheel)

        (motor, hinge, wheel) = make_wheel_assembly("front_right",
                                                    xs - 1.5, ys + motor_y, zs,
                                                    flip=-1.0)
        add_compound_request.body.append(motor)
        add_compound_request.constraint.append(hinge)
        add_compound_request.soft_body.append(wheel)

        (motor, hinge, wheel) = make_wheel_assembly("back_right",
                                                    xs + 1.5, ys + motor_y, zs,
                                                    flip=-1.0)
        add_compound_request.body.append(motor)
        add_compound_request.constraint.append(hinge)
        add_compound_request.soft_body.append(wheel)

        try:
            add_compound_response = self.add_compound(add_compound_request)
            rospy.loginfo(add_compound_response)
        except rospy.service.ServiceException as e:
            rospy.logerr(e)


if __name__ == '__main__':
    rospy.init_node('soft_vehicle')
    soft_vehicle = SoftVehicle()
