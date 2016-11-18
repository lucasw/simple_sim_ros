#!/usr/bin/env python
# Copyright Lucas Walter 2016

import copy
import math
import rospy
import tf

from bullet_server.msg import Body, Constraint, Face, Link, Material, Node, SoftBody, Tetra
from bullet_server.srv import *

def make_rigid_box(name, mass, xs, ys, zs, wd, ln, ht):
    body = Body()
    body.name = name
    body.type = Body.BOX
    body.mass = mass
    body.pose.position.x = xs
    body.pose.position.y = ys
    body.pose.position.z = zs
    body.pose.orientation.w = 1.0
    body.scale.x = wd
    body.scale.y = ln
    body.scale.z = ht
    return body

def make_rigid_cylinder(name, mass, xs, ys, zs, radius, thickness,
                        roll=math.pi/2.0, pitch=0, yaw=0):
    # make the top cylinder plate
    body = Body()
    body.name = name
    body.mass = mass
    rot90 = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    body.pose.orientation.x = rot90[0]
    body.pose.orientation.y = rot90[1]
    body.pose.orientation.z = rot90[2]
    body.pose.orientation.w = rot90[3]
    body.pose.position.x = xs
    body.pose.position.y = ys
    body.pose.position.z = zs
    body.type = Body.CYLINDER
    body.scale.x = radius
    body.scale.y = thickness
    body.scale.z = radius
    return body

def make_soft_cube(name, xs, ys, zs):
    body = SoftBody()
    body.name = name

    mass = 0.5
    n1 = Node()
    n1.mass = mass
    n1.position.x = xs
    n1.position.y = ys
    n1.position.z = zs
    body.node.append(n1)

    n1 = Node()
    n1.mass = mass
    n1.position.x = xs + 1.0
    n1.position.y = ys
    n1.position.z = zs
    body.node.append(n1)

    n1 = Node()
    n1.mass = mass
    n1.position.x = xs
    n1.position.y = ys + 1.0
    n1.position.z = zs
    body.node.append(n1)

    n1 = Node()
    n1.mass = mass
    n1.position.x = xs
    n1.position.y = ys
    n1.position.z = zs + 1.0
    body.node.append(n1)

    l1 = Link()
    l1.node_indices[0] = 0
    l1.node_indices[1] = 1
    body.link.append(l1)

    l1 = Link()
    l1.node_indices[0] = 0
    l1.node_indices[1] = 2
    body.link.append(l1)

    l1 = Link()
    l1.node_indices[0] = 0
    l1.node_indices[1] = 3
    body.link.append(l1)

    l1 = Link()
    l1.node_indices[0] = 1
    l1.node_indices[1] = 2
    body.link.append(l1)

    l1 = Link()
    l1.node_indices[0] = 1
    l1.node_indices[1] = 3
    body.link.append(l1)

    l1 = Link()
    l1.node_indices[0] = 2
    l1.node_indices[1] = 3
    body.link.append(l1)

    mat = Material()
    mat.kLST = 0.2
    mat.kVST = 0.1
    mat.kAST = 0.1
    body.material.append(mat)

    return body

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

        chassis = make_rigid_box("chassis", 1.0, xs, ys, zs, 0.6, 0.5, 0.3)
        add_compound_request.body.append(chassis)

        motor_y = 0.8
        left_motor = make_rigid_cylinder("left_motor", 0.3,
                                         xs, ys - motor_y, zs, 0.3, 0.1,
                                         0, 0, 0)
        add_compound_request.body.append(left_motor)
        right_motor = make_rigid_cylinder("right_motor", 0.3,
                                          xs, ys + motor_y, zs, 0.3, 0.1,
                                          0, 0, 0)
        add_compound_request.body.append(right_motor)

        hinge = Constraint()
        hinge.name = "left_motor_hinge"
        hinge.body_a = "chassis"
        hinge.body_b = "left_motor"
        hinge.type = Constraint.HINGE
        hinge.pivot_in_a.x = 0
        hinge.pivot_in_a.y = -motor_y
        hinge.pivot_in_a.z = 0
        hinge.pivot_in_b.x = 0
        hinge.pivot_in_b.y = 0
        hinge.pivot_in_b.z = 0
        hinge.axis_in_a.x = 0
        hinge.axis_in_a.y = 1.0
        hinge.axis_in_a.z = 0
        hinge.axis_in_b.x = 0
        hinge.axis_in_b.y = 1.0
        hinge.axis_in_b.z = 0
        hinge.max_motor_impulse = 5000.0
        add_compound_request.constraint.append(hinge)

        hinge2 = copy.deepcopy(hinge)
        hinge2.name = "right_motor_hinge"
        hinge2.body_b = "right_motor"
        hinge2.pivot_in_a.y = motor_y
        add_compound_request.constraint.append(hinge2)

        left_wheel = make_soft_cube("left_wheel", xs, ys + 3, zs)
        add_compound_request.soft_body.append(left_wheel)
        right_wheel = make_soft_cube("right_wheel", xs, ys - 3, zs)
        add_compound_request.soft_body.append(right_wheel)

        try:
            add_compound_response = self.add_compound(add_compound_request)
            rospy.loginfo(add_compound_response)
        except rospy.service.ServiceException as e:
            rospy.logerr(e)

if __name__ == '__main__':
    rospy.init_node('soft_vehicle')
    soft_vehicle = SoftVehicle()
