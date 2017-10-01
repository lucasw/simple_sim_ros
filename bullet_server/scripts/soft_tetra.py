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

        length = rospy.get_param("~length", 0.1)
        width = rospy.get_param("~width", 0.1)
        height = rospy.get_param("~height", 0.1)

        nx = rospy.get_param("~nx", 3)
        ny = rospy.get_param("~ny", 3)
        nz = rospy.get_param("~nz", 4)

        lx = length / float(nx - 1)
        ly = width / float(ny - 1)
        lz = height / float(nz - 1)
        volume = lx * ly * lz

        total_mass = rospy.get_param("~total_mass", 0.5)
        num_cubes = (nx - 1) * (ny - 1) * (nz - 1)
        tetras_per_cube = 5
        node_mass = total_mass / float(num_cubes * tetras_per_cube)
        if True:
            body = make_soft_tetra_cube("soft_tetra_cube",
                                        node_mass,
                                        xs, ys, zs,
                                        lx, ly, lz,
                                        nx, ny, nz,
                                        1.0)
        else:
            body = make_soft_cube("soft_tetra_cube",
                                  node_mass,
                                  xs, ys, zs,
                                  lx,  # ly, lz,
                                  nx, ny, nz,
                                  1)

        body.material[0].kLST = 0.79
        # these do nothing
        body.material[0].kAST = 0.0
        body.material[0].kVST = 0.9

        # body.margin = lx
        body.material[0].bending_distance = 2
        body.randomize_constraints = False  # True
        body.k_clusters = 4
        # volume preserving?  Doesn't seem to work
        body.config.kVC = 1e9 * volume
        body.config.kDP = 0.9
        body.config.kDF = 0.999
        # body.config.kMT = 0.5
        # pressure preserving?
        # setting this to anything with no faces (?) will result in nans
        body.config.kPR = 90000.0 * lx * ly * lz
        print body

        add_compound_request.soft_body.append(body)

        if False:
            cyl = make_rigid_cylinder("cyl", 0.1,
                                      0, 0, 1.2,
                                      0.8,
                                      0.1,
                                      math.pi/2.0, 0, 0)
            add_compound_request.body.append(cyl)

        try:
            add_compound_response = self.add_compound(add_compound_request)
            rospy.loginfo(add_compound_response)
        except rospy.service.ServiceException as e:
            rospy.logerr(e)

if __name__ == '__main__':
    rospy.init_node('soft_tetra_demo')
    soft_body_demo = SoftBodyDemo()
