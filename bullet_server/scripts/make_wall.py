#!/usr/bin/env python
# Copyright (c) 2016 Lucas Walter
# December 2016

# create a wall of blocks of param defined size

import copy
import numpy
import rospy
import tf
import tf2_ros

from bullet_server.msg import Body, Face, Link, Material, Node, SoftBody, SoftConfig, Tetra
from bullet_server.srv import *

if __name__ == '__main__':
    rospy.init_node('imarker_spawn')

    rospy.wait_for_service('add_compound')
    add_compound = rospy.ServiceProxy('add_compound', AddCompound)

    num_x = rospy.get_param("~num_x", 10)
    num_y = rospy.get_param("~num_y", 10)
    size_x = rospy.get_param("~size_x", 0.3)
    size_y = rospy.get_param("~size_y", 0.3)
    size_z = rospy.get_param("~size_z", 0.3)
    mass = rospy.get_param("~mass", 1.0)
    margin_x = rospy.get_param("~margin_x", 0.01)
    margin_z = rospy.get_param("~margin_z", 0.00001)

    add_compound_request = AddCompoundRequest()
    add_compound_request.remove = False

    for j in range(num_y):
        for i in range(num_x):
            body = Body()
            body.name = "wall_box_" + str(i) + "_" + str(j)
            body.mass = mass
            x_offset = -num_x / 2.0 * size_x + (j % 2) * size_x / 3.0
            body.pose.position.x = x_offset + i * (size_x + margin_x)
            body.pose.position.y = 2.0
            body.pose.position.z = size_z / 2.0 + margin_z + j * (size_z + margin_z)
            body.pose.orientation.w = 1.0
            body.type = Body.BOX
            body.scale.x = size_x / 2.0
            body.scale.y = size_y / 2.0
            body.scale.z = size_z / 2.0

            add_compound_request.body.append(body)

    try:
        add_compound_response = add_compound(add_compound_request)
        rospy.loginfo(add_compound_response)
    except rospy.service.ServiceException as e:
        rospy.logerr(e)
