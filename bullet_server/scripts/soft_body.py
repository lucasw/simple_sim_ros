#!/usr/bin/env python
# Copyright Lucas Walter 2016

import rospy

from bullet_server.msg import Face, Link, Node, SoftBody, Tetra
from bullet_server.srv import *


class SoftBodyDemo:
    def __init__(self):
        rospy.wait_for_service('add_compound')
        self.add_compound = rospy.ServiceProxy('add_compound', AddCompound)
        add_compound_request = AddCompoundRequest()
        add_compound_request.remove = rospy.get_param('~remove', False)

        zs = 1.0

        body = SoftBody()

        n1 = Node()
        n1.mass = 0.1
        n1.position.x = 0.0
        n1.position.y = 0.0
        n1.position.z = zs
        body.node.append(n1)

        n1 = Node()
        n1.mass = 0.1
        n1.position.x = 1.0
        n1.position.y = 0.0
        n1.position.z = zs
        body.node.append(n1)

        n1 = Node()
        n1.mass = 0.1
        n1.position.x = 0.0
        n1.position.y = 1.0
        n1.position.z = zs
        body.node.append(n1)

        n1 = Node()
        n1.mass = 0.1
        n1.position.x = 0.0
        n1.position.y = 0.0
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
        l1.node_indices[1] = 3
        body.link.append(l1)

        l1 = Link()
        l1.node_indices[0] = 2
        l1.node_indices[1] = 3
        body.link.append(l1)

        add_compound_request.soft_body.append(body)

        try:
            add_compound_response = self.add_compound(add_compound_request)
            rospy.loginfo(add_compound_response)
        except rospy.service.ServiceException as e:
            rospy.logerr(e)

if __name__ == '__main__':
    rospy.init_node('soft_body_demo')
    soft_body_demo = SoftBodyDemo()
