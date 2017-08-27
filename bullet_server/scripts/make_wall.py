#!/usr/bin/env python
# Copyright (c) 2016 Lucas Walter
# December 2016

# create a wall of blocks of param defined size

import copy
import math
import numpy
import rospy
import tf

from bullet_server.msg import Body, Face, Link, Material, Node, SoftBody, SoftConfig, Tetra
from bullet_server.srv import *

if __name__ == '__main__':
    rospy.init_node('imarker_spawn')

    rospy.wait_for_service('add_compound')
    add_compound = rospy.ServiceProxy('add_compound', AddCompound)

    x = rospy.get_param("~x", 0.0)
    y = rospy.get_param("~y", 5.0)
    num_x = rospy.get_param("~num_x", 12)
    num_y = rospy.get_param("~num_y", 10)
    size_x = rospy.get_param("~size_x", 0.4)
    size_y = rospy.get_param("~size_y", 0.3)
    size_z = rospy.get_param("~size_z", 0.15)
    mass = rospy.get_param("~mass", 0.2)
    margin_x = rospy.get_param("~margin_x", 0.00001)
    margin_z = rospy.get_param("~margin_z", 0.01)

    add_compound_request = AddCompoundRequest()
    add_compound_request.remove = False

    if False:
        for j in range(num_y):
            for i in range(num_x):
                body = Body()
                body.name = "wall_box_" + str(i) + "_" + str(j)
                body.mass = mass
                body.type = Body.BOX
                body.scale.x = size_x / 2.0
                body.scale.y = size_y / 2.0
                body.scale.z = size_z / 2.0

                x_offset = -num_x / 2.0 * size_x + (j % 2) * size_x / 3.0
                body.pose.position.x = x + x_offset + i * (size_x + margin_x)
                body.pose.position.y = y
                body.pose.position.z = size_z / 2.0 + margin_z + j * (size_z + margin_z)
                body.pose.orientation.w = 1.0

                add_compound_request.body.append(body)
    else:
        # make a cylinder wall
        circumference = size_x * num_x
        # TODO(lucasw) this isn't quite right
        radius = circumference / (2.0 * math.pi) + size_y / 2.0
        for j in range(num_y):
            for i in range(num_x):
                body = Body()
                body.name = "wall_box_" + str(i) + "_" + str(j)
                body.mass = mass
                body.type = Body.BOX
                body.scale.x = size_x / 2.0
                body.scale.y = size_y / 2.0
                body.scale.z = size_z / 2.0

                angle = (float(i) + (j % 2) * 0.5) / float(num_x) * 2.0 * math.pi
                body.pose.position.x = x + radius * math.cos(angle)
                body.pose.position.y = y + radius * math.sin(angle)
                # body.pose.position.z = size_z / 2.0 + margin_z + j * (size_z + margin_z)
                body.pose.position.z = margin_z + j * (size_z + margin_z)
                quat = tf.transformations.quaternion_from_euler(0, 0, angle + math.pi/2.0)
                body.pose.orientation.x = quat[0]
                body.pose.orientation.y = quat[1]
                body.pose.orientation.z = quat[2]
                body.pose.orientation.w = quat[3]

                add_compound_request.body.append(body)

    try:
        add_compound_response = add_compound(add_compound_request)
        rospy.loginfo(add_compound_response)
    except rospy.service.ServiceException as e:
        rospy.logerr(e)
