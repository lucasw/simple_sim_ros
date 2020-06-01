#!/usr/bin/env python
# generate an icosphere soft body

import copy
import math
import rospy

from bullet_server.msg import Face, Link, Material, Node, SoftBody
from bullet_server.srv import *
from bullet_server.utility import *
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


class Icosphere:
    def __init__(self):
        self.pub = rospy.Publisher("marker", Marker, queue_size=2)

        radius = rospy.get_param("~radius", 0.5)
        levels = rospy.get_param("~levels", 3)
        px = rospy.get_param("~x", 0.0)
        name = rospy.get_param("~name", "ball")

        body = SoftBody()
        body.name = name
        body.pose.position.x = px
        body.pose.position.z = 0.2
        body.pose.orientation.w = 1.0

        # from http://blog.andreaskahler.com/2009/06/creating-icosphere-mesh-in-code.html
        icost = (1.0 + math.sqrt(5.0)) / 2.0
        tmp_radius = math.sqrt(1.0 + icost * icost)
        scale = radius / tmp_radius

        self.pts = []
        self.subpts = {}

        pt = Point()
        self.pts.append(Point(-1, icost, 0))
        self.pts.append(Point(1, icost, 0))
        self.pts.append(Point(-1, -icost, 0))
        self.pts.append(Point(1, -icost, 0))

        self.pts.append(Point(0, -1, icost))
        self.pts.append(Point(0, 1, icost))
        self.pts.append(Point(0, -1, -icost))
        self.pts.append(Point(0, 1, -icost))

        self.pts.append(Point(icost, 0, -1))
        self.pts.append(Point(icost, 0, 1))
        self.pts.append(Point(-icost, 0, -1))
        self.pts.append(Point(-icost, 0, 1))

        # first construct an icosohedron
        # rospy.loginfo(ico)

        faces = []
        faces.append([0, 11, 5])
        faces.append([0, 5, 1])
        faces.append([0, 1, 7])
        faces.append([0, 7, 10])
        faces.append([0, 10, 11])

        if True:
            faces.append([1, 5, 9])
            faces.append([5, 11, 4])
            faces.append([11, 10, 2])
            faces.append([10, 7, 6])
            faces.append([7, 1, 8])

            faces.append([3, 9, 4])
            faces.append([3, 4, 2])
            faces.append([3, 2, 6])
            faces.append([3, 6, 8])
            faces.append([3, 8, 9])

            faces.append([4, 9, 5])
            faces.append([2, 4, 11])
            faces.append([6, 2, 10])
            faces.append([8, 6, 7])
            faces.append([9, 8, 1])

        for i in range(levels):
            faces = self.subdivide(faces)

        height = rospy.get_param("~height", radius * 1.1)

        print len(self.pts)
        # scale all points to be on sphere of desired radius
        for i in range(len(self.pts)):
            pt = self.pts[i]
            scale = radius / math.sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z)
            pt.x *= scale
            pt.y *= scale
            pt.z *= scale
            pt.z += height
            self.pts[i] = pt

        if False:
            marker = Marker()
            marker.header.frame_id = rospy.get_param("~frame_id", "map")
            marker.ns = name
            marker.id = 1
            marker.type = Marker.TRIANGLE_LIST
            marker.action = Marker.ADD
            marker.pose.position.x = px
            marker.pose.orientation.w = 1.0
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0

            marker.color.a = 1.0
            marker.color.r = rospy.get_param("~r", 1.0)
            marker.color.g = rospy.get_param("~g", 1.0)
            marker.color.b = rospy.get_param("~b", 1.0)

            for face in faces:
                print face
                marker.points.append(self.pts[face[0]])
                marker.points.append(self.pts[face[1]])
                marker.points.append(self.pts[face[2]])

            rospy.sleep(0.5)
            self.pub.publish(marker)
            rospy.sleep(0.5)

        # make nodes and links and faces in SoftBody
        total_mass = rospy.get_param("~mass", 2.0)
        node_mass = total_mass / len(self.pts)
        for pt in self.pts:
            node = Node()
            node.position.x = pt.x
            node.position.y = pt.y
            node.position.z = pt.z
            node.mass = node_mass
            body.node.append(node)

        # center node
        # This doesn't work well, would rather have a sphere
        # built out of a layer of tetras
        if False:
            node = Node()
            node.mass = node_mass
            node.position.x = 0
            node.position.y = 0
            node.position.z = height
            body.node.append(node)
            center_ind = len(body.node) - 1
            for i in range(0, len(body.node), 10):
                link = Link()
                link.node_indices[0] = i
                link.node_indices[1] = center_ind
                body.link.append(link)

        link_map = {}
        for i in range(len(faces)):
            face = Face()
            for j in range(3):
                face.node_indices[j] = faces[i][j]
                ind1 = faces[i][j]
                ind2 = faces[i][(j + 1) % 3]
                # prevent duplicate links
                ind_in_map = ((ind1 in link_map.keys() and ind2 in link_map[ind1].keys()) or
                              (ind2 in link_map.keys() and ind1 in link_map[ind2].keys()))
                if ind_in_map:
                    continue
                link = Link()
                link.node_indices[0] = ind1
                link.node_indices[1] = ind2
                if ind1 not in link_map.keys():
                    link_map[ind1] = {}
                link_map[ind1][ind2] = True
                body.link.append(link)
            body.face.append(face)

        mat = Material()
        mat.kLST = 0.9
        mat.bending_distance = 2
        body.material.append(mat)

        body.config = make_soft_config()
        body.config.kDF = 1.0
        body.config.kDP = 0.004
        body.config.kDG = 0.005
        body.config.kPR = rospy.get_param("~pressure", 700.0)  # pressure coefficient
        body.config.kMT = 0.9
        body.config.maxvolume = 0.5

        body.margin = radius / 40.0
        body.k_clusters = 8
        body.randomize_constraints = True

        rospy.wait_for_service('add_compound')
        self.add_compound = rospy.ServiceProxy('add_compound', AddCompound)
        add_compound_request = AddCompoundRequest()
        add_compound_request.remove = False  # rospy.get_param('~remove', False)
        add_compound_request.soft_body.append(body)

        rospy.loginfo(add_compound_request)
        try:
            add_compound_response = self.add_compound(add_compound_request)
            rospy.loginfo(add_compound_response)
        except rospy.service.ServiceException as e:
            rospy.logerr(e)

    def make_triangle(self, src, dst, i0, i1, i2, subdiv=1):
        if subdiv == 0:
            dst.append(src[i0])
            # dst.append(src[i1])

            dst.append(src[i1])
            # dst.append(src[i2])

            dst.append(src[i2])
            # dst.append(src[i0])
            return

        dst.extend(self.sub_tri(src[i0], src[i1], src[i2]))

    def mid_point(self, i1, i2):
        if i1 in self.subpts.keys() and i2 in self.subpts[i1].keys():
            return self.subpts[i1][i2]
        if i2 in self.subpts.keys() and i1 in self.subpts[i2].keys():
            return self.subpts[i2][i1]
        v1 = self.pts[i1]
        v2 = self.pts[i2]
        pt = Point(
                   (v1.x + v2.x) / 2.0,
                   (v1.y + v2.y) / 2.0,
                   (v1.z + v2.z) / 2.0)

        self.pts.append(pt)
        ind = len(self.pts) - 1
        if i1 not in self.subpts.keys():
            self.subpts[i1] = {}
        self.subpts[i1][i2] = ind
        return ind

    def subdivide(self, faces):  # v1, v2, v3, levels=1):
        #        v2
        #
        #     a     b
        #
        #   v1   c    v3
        subfaces = []
        for fc in faces:
            ai = self.mid_point(fc[0], fc[1])
            bi = self.mid_point(fc[1], fc[2])
            ci = self.mid_point(fc[2], fc[0])

            subfaces.extend([[fc[0], ai, ci],
                             [ci, ai, bi],
                             [ci, bi, fc[2]],
                             [ai, fc[1], bi],
                             ])
        return (subfaces)


if __name__ == '__main__':
    rospy.init_node('icosphere')
    icosphere = Icosphere()
