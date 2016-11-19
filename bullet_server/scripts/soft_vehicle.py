#!/usr/bin/env python
# Copyright Lucas Walter 2016

import copy
import math
import rospy
import tf

from bullet_server.msg import Anchor, Body, Constraint, Face, Link, Material, Node, SoftBody, Tetra
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

def make_soft_cube(name, node_mass, xs, ys, zs, ln,
                   nx=4, ny=4, nz=4):
    body = SoftBody()
    body.name = name

    for i in range(nx):
        for j in range(ny):
            for k in range(nz):
                n1 = Node()
                n1.mass = node_mass
                n1.position.x = xs + (i - nx/2 + 0.5) * ln
                n1.position.y = ys + (j - ny/2 + 0.5) * ln
                n1.position.z = zs + (k - nz/2 + 0.5) * ln
                body.node.append(n1)

    for ind1 in range(len(body.node)):
        for i in range(2):
            for j in range(2):
                for k in range(2):
                    if i == 0 and j == 0 and k == 0:
                        continue
                    ind2 = ind1 + i * ny * nz + j * nz + k
                    if ind2 < len(body.node):
                        l1 = Link()
                        l1.node_indices[0] = ind1
                        l1.node_indices[1] = ind2
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
            ground_radius = 50
            ground_thickness = 1.0
            ground_mass = 0.0
            ground = make_rigid_cylinder("ground", ground_mass,
                                         0, 0, -ground_thickness, ground_radius,
                                         ground_thickness,
                                         math.pi/2.0, 0, 0)
            add_compound_request.body.append(ground)

        chassis = make_rigid_box("chassis", 1.5, xs, ys, zs, 1.2, 0.5, 0.3)
        add_compound_request.body.append(chassis)

        motor_mass = 0.2
        motor_thickness = 0.1
        motor_radius = 0.2
        motor_y = 0.8
        left_motor = make_rigid_cylinder("left_motor", motor_mass,
                                         xs, ys - motor_y, zs, motor_radius, motor_thickness,
                                         0, 0, 0)
        add_compound_request.body.append(left_motor)
        right_motor = make_rigid_cylinder("right_motor", 0.3,
                                          xs, ys + motor_y, zs, motor_radius, motor_thickness,
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
        # These have to be < -pi and > pi to be unlimited
        hinge.lower_ang_lim = -3.2  # -math.pi
        hinge.upper_ang_lim = 3.2  # math.pi
        hinge.max_motor_impulse = 25000.0
        add_compound_request.constraint.append(hinge)

        hinge2 = copy.deepcopy(hinge)
        hinge2.name = "right_motor_hinge"
        hinge2.body_b = "right_motor"
        hinge2.pivot_in_a.y = motor_y
        add_compound_request.constraint.append(hinge2)

        wheel_offset = 1.5
        nx = 4
        ny = 4
        nz = 4
        soft_length = 0.4
        node_mass = 0.1
        left_wheel = make_soft_cube("left_wheel", node_mass, xs, ys - wheel_offset, zs, soft_length,
                                    nx, ny, nz)

        right_wheel = make_soft_cube("right_wheel", node_mass, xs, ys + wheel_offset, zs, soft_length,
                                     nx, ny, nz)
        # attach the rigid motor wheel to the soft wheel
        if True:
            for i in range(2):
                for j in range(3):
                    for k in range(2):
                        anchor = Anchor()
                        anchor.disable_collision_between_linked_bodies = False
                        anchor.influence = 1.0

                        xi = i + 1  # int(nx/2)
                        yi = j + 1  # + int(ny/2)
                        zi = k + 1  # int(nz/2)
                        ind = xi * ny * nz + yi * nz + zi
                        # the weaker the influence, the longer the spring between
                        # the local_pivot and the node
                        if ind < len(left_wheel.node):
                            anchor.node_index = ind
                            anchor.rigid_body_name = "left_motor"
                            anchor.local_pivot.x = left_wheel.node[ind].position.x - xs  # left_motor.pose.position.x
                            anchor.local_pivot.y = left_wheel.node[ind].position.y - ys + 0.8  # left_motor.pose.position.y
                            anchor.local_pivot.z = left_wheel.node[ind].position.z - zs  # left_motor.pose.position.z
                            left_wheel.anchor.append(anchor)

                        anchor = copy.deepcopy(anchor)

                        xi = i + 1  # int(nx/2)
                        yi = j  # + int(ny/2)
                        zi = k + 1  # int(nz/2)
                        ind = xi * ny * nz + yi * nz + zi
                        if ind >= len(left_wheel.node):
                            continue

                        anchor.node_index = ind
                        anchor.rigid_body_name = "right_motor"
                        anchor.local_pivot.x = right_wheel.node[ind].position.x - xs  # left_motor.pose.position.x
                        anchor.local_pivot.y = right_wheel.node[ind].position.y - ys - 0.8  # left_motor.pose.position.y
                        anchor.local_pivot.z = right_wheel.node[ind].position.z - zs  # left_motor.pose.position.z
                        print anchor.local_pivot
                        anchor.disable_collision_between_linked_bodies = False
                        # the weaker the influence, the longer the spring between
                        # the local_pivot and the node
                        anchor.influence = 1.0

                        right_wheel.anchor.append(anchor)


        add_compound_request.soft_body.append(left_wheel)

        add_compound_request.soft_body.append(right_wheel)

        try:
            add_compound_response = self.add_compound(add_compound_request)
            rospy.loginfo(add_compound_response)
        except rospy.service.ServiceException as e:
            rospy.logerr(e)

if __name__ == '__main__':
    rospy.init_node('soft_vehicle')
    soft_vehicle = SoftVehicle()
