#!/usr/bin/env python
# Copyright Lucas Walter 2016

import math
import rospy
import tf

from bullet_server.msg import Body, Face, Link, Material, Node, SoftBody, SoftConfig, Tetra
from bullet_server.srv import *
from bullet_server.utility import *


class SoftBodyDemo:
    def __init__(self):
        rospy.wait_for_service('add_compound')
        self.add_compound = rospy.ServiceProxy('add_compound', AddCompound)
        add_compound_request = AddCompoundRequest()
        add_compound_request.remove = rospy.get_param('~remove', False)

        xs = rospy.get_param("~x", 0.0)
        ys = rospy.get_param("~y", 0.0)
        zs = rospy.get_param("~z", 1.0)

        body = SoftBody()
        body.name = "pyramid"
        body.config = make_soft_config()

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
        mat.kLST = 0.15
        mat.kVST = 0.1
        mat.kAST = 0.1
        body.material.append(mat)

        body.k_clusters = 8
        body.margin = 0.05

        add_compound_request.soft_body.append(body)

        if False:
            # make the top cylinder plate
            top_plate = Body()
            top_plate.name = "top_plate"
            top_plate.mass = 0.3
            rot90 = tf.transformations.quaternion_from_euler(math.pi/2.0, 0, 0)
            radius = 1.5
            thickness = 0.5
            top_plate.pose.orientation.x = rot90[0]
            top_plate.pose.orientation.y = rot90[1]
            top_plate.pose.orientation.z = rot90[2]
            top_plate.pose.orientation.w = rot90[3]
            top_plate.pose.position.x = 0
            top_plate.pose.position.y = 0
            top_plate.pose.position.z = 1.7
            top_plate.type = Body.CYLINDER
            top_plate.scale.x = radius
            top_plate.scale.y = thickness
            top_plate.scale.z = radius
            add_compound_request.body.append(top_plate)

        rospy.loginfo(add_compound_request)
        try:
            add_compound_response = self.add_compound(add_compound_request)
            rospy.loginfo(add_compound_response)
        except rospy.service.ServiceException as e:
            rospy.logerr(e)


if __name__ == '__main__':
    rospy.init_node('soft_body_demo')
    soft_body_demo = SoftBodyDemo()
