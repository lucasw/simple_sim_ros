#!/usr/bin/env python
# Copyright Lucas Walter 2016

import copy
import math
import rospy
import tf

from bullet_server.msg import Anchor, Body, Constraint, Face, Link, Material, Node, SoftBody, Tetra
from bullet_server.srv import *

def make_rigid_box(name, mass, xs, ys, zs, wd, ln, ht,
                   roll=0, pitch=0, yaw=0):
    body = Body()
    body.name = name
    body.type = Body.BOX
    body.mass = mass
    body.pose.position.x = xs
    body.pose.position.y = ys
    body.pose.position.z = zs
    rot90 = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    body.pose.orientation.x = rot90[0]
    body.pose.orientation.y = rot90[1]
    body.pose.orientation.z = rot90[2]
    body.pose.orientation.w = rot90[3]

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
                   nx=4, ny=4, nz=4, flip=1.0):
    body = SoftBody()
    body.name = name

    for i in range(nx):
        for j in range(ny):
            for k in range(nz):
                n1 = Node()
                n1.mass = node_mass
                n1.position.x = xs + (i - nx/2 + 0.5) * ln * flip
                n1.position.y = ys + (j - ny/2 + 0.5) * ln * flip
                n1.position.z = zs + (k - nz/2 + 0.5) * ln * flip
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
    mat.kLST = 0.25
    mat.kVST = 0.1
    mat.kAST = 0.1
    body.material.append(mat)

    return body

def make_wheel_assembly(prefix, xs, ys, zs, flip=1.0):
    motor_mass = 0.2
    motor_thickness = 0.1
    motor_radius = 0.2
    motor_y = 0.8
    motor = make_rigid_cylinder(prefix + "_motor", motor_mass,
                                xs, ys - motor_y * flip, zs,
                                motor_radius, motor_thickness,
                                0, 0, 0)

    hinge = Constraint()
    hinge.name = prefix + "_motor_hinge"
    hinge.body_a = "chassis"
    hinge.body_b = prefix + "_motor"
    hinge.type = Constraint.HINGE
    hinge.pivot_in_a.x = xs
    hinge.pivot_in_a.y = -motor_y * flip
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

    wheel_offset = 1.5
    nx = 4
    ny = 4
    nz = 4
    soft_length = 0.4
    node_mass = 0.1
    wheel = make_soft_cube(prefix + "wheel", node_mass,
                           xs, ys - wheel_offset * flip, zs, soft_length,
                           nx, ny, nz,
                           flip)

    # attach the rigid motor wheel to the soft wheel
    for i in range(2):
        for j in range(3):
            for k in range(2):
                xi = i + 1  # int(nx/2) - 1
                yi = j + 1  # int(ny/2) - 1
                zi = k + 1  # int(nz/2) - 1
                ind = xi * ny * nz + yi * nz + zi
                if ind < len(wheel.node):
                    anchor = Anchor()
                    anchor.disable_collision_between_linked_bodies = False
                    # the weaker the influence, the longer the spring between
                    # the local_pivot and the node
                    # TODO(lucasw) probably should have more anchors,
                    # but weaken the influence with distance from the wheel
                    # axle
                    anchor.influence = 1.0

                    anchor.node_index = ind
                    anchor.rigid_body_name = prefix + "_motor"
                    anchor.local_pivot.x = wheel.node[ind].position.x - xs
                    anchor.local_pivot.y = wheel.node[ind].position.y - ys + 0.8 * flip
                    anchor.local_pivot.z = wheel.node[ind].position.z - zs
                    wheel.anchor.append(anchor)

    return (motor, hinge, wheel)

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
                                           -6.0, 1.0, -ground_thickness + 0.5,
                                           3.0,
                                           ground_thickness,
                                           math.pi/2.0, 0, 0)
            add_compound_request.body.append(obstacle)
            obstacle = make_rigid_box("obstacle2", ground_mass,
                                      -12.0, 1.5, 0.5,
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
