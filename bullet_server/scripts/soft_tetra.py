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

        ground_radius = 10
        ground_thickness = 1.0
        ground_mass = 0.0
        ground = make_rigid_cylinder("ground", ground_mass,
                                     0, 0, -ground_thickness,
                                     ground_radius,
                                     ground_thickness,
                                     math.pi/2.0, 0, 0)
        add_compound_request.body.append(ground)

        body = make_soft_tetra_cube("soft_tetra_cube", 0.1,
                                    xs, ys, zs, 4.0)

        body.material[0].kLST = 0.2
        # these do nothing
        body.material[0].kAST = 0.0
        body.material[0].kVST = 0.9

        # volume preserviing?
        body.config.kVC = 200
        # pressure preserving?
        # body.config.kPR = 2500
        print body

        add_compound_request.soft_body.append(body)

        cyl = make_rigid_cylinder("cyl", 0.1,
                                  0, 0, 1.2,
                                  0.8,
                                  0.1,
                                  math.pi/2.0, 0, 0)
        # add_compound_request.body.append(cyl)

        try:
            add_compound_response = self.add_compound(add_compound_request)
            rospy.loginfo(add_compound_response)
        except rospy.service.ServiceException as e:
            rospy.logerr(e)

if __name__ == '__main__':
    rospy.init_node('soft_tetra_demo')
    soft_body_demo = SoftBodyDemo()
