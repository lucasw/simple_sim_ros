#!/usr/bin/env python
# generate an strand - make rope/string that is taut and vibrates when plucked

import copy
import math
import rospy

from bullet_server.msg import Anchor, Face, Link, Material, Node, SoftBody
from bullet_server.srv import *
from bullet_server.utility import *
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


class Strand:
    def __init__(self):
        length = rospy.get_param("~length", 0.5)
        segments = rospy.get_param("~segments", 20)
        py = rospy.get_param("~y", 0.0)
        name = rospy.get_param("~name", "strand")

        pz = 0.2
        body = SoftBody()
        body.name = name
        body.pose.position.y = 0
        body.pose.position.z = 0
        body.pose.orientation.w = 1.0

        # make nodes and links and faces in SoftBody
        total_mass = rospy.get_param("~mass", 1.0)
        node_mass = total_mass / float(segments)
        node_length = length / float(segments)
        x = 0
        for i in range(segments):
            node = Node()
            node.position.x = x
            node.position.y = py + 0.15
            node.position.z = pz
            node.mass = node_mass
            body.node.append(node)
            x += node_length
            if i > 0:
                link = Link()
                link.node_indices[0] = i - 1
                link.node_indices[1] = i
                body.link.append(link)

        # TODO(lucasw) need to anchor first and last elements to static (or possibly kinematic) rigid bodies

        # TODO(lucasw) it would be nice to be able to tune these live
        mat = Material()
        mat.kLST = 0.9
        mat.kAST = 0.1
        # TODO(lucasw)
        mat.bending_distance = 2
        body.material.append(mat)

        body.config = make_soft_config()
        body.config.kDF = 1.0
        body.config.kDP = 0.004
        body.config.kDG = 0.005
        # body.config.kPR = rospy.get_param("~pressure", 700.0)  # pressure coefficient
        body.config.kMT = 0.1
        body.config.maxvolume = 0.5

        body.margin = length / 40.0
        body.k_clusters = 8
        body.randomize_constraints = True

        anchor0_body = make_rigid_box(name + "_anchor0", 0.0,
                                      body.node[0].position.x - node_length, py, pz,
                                      node_length, node_length, node_length * 10.0,
                                      roll=0, pitch=0, yaw=0)
        anchor0 = Anchor()
        anchor0.node_index = 0
        anchor0.rigid_body_name = anchor0_body.name
        anchor0.influence = 0.05
        anchor0.disable_collision_between_linked_bodies = True

        anchor1_body = make_rigid_box(name + "_anchor1", 0.0,
                                      body.node[-1].position.x + node_length, py, pz,
                                      node_length, node_length, node_length * 10.0,
                                      roll=0, pitch=0, yaw=0)
        anchor1 = Anchor()
        anchor1.node_index = len(body.node) - 1
        anchor1.rigid_body_name = anchor1_body.name
        anchor1.influence = anchor0.influence
        anchor1.disable_collision_between_linked_bodies = True

        body.anchor.append(anchor0)
        body.anchor.append(anchor1)

        add_compound_request = AddCompoundRequest()
        add_compound_request.remove = False  # rospy.get_param('~remove', False)
        add_compound_request.soft_body.append(body)
        add_compound_request.body.append(anchor0_body)
        add_compound_request.body.append(anchor1_body)

        rospy.wait_for_service('add_compound')
        self.add_compound = rospy.ServiceProxy('add_compound', AddCompound)
        rospy.loginfo(add_compound_request)
        try:
            add_compound_response = self.add_compound(add_compound_request)
            rospy.loginfo(add_compound_response)
        except rospy.service.ServiceException as e:
            rospy.logerr(e)


if __name__ == '__main__':
    rospy.init_node('strand')
    strand = Strand()
