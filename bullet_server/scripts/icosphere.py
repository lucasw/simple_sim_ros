#!/usr/bin/env python
# generate an icosphere soft body

import copy
import math
import rospy

from bullet_server.msg import Face, Node, SoftBody
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


class Icosphere:
    def __init__(self):
        self.pub = rospy.Publisher("marker", Marker, queue_size=2)

        radius = rospy.get_param("~radius", 0.5)
        levels = rospy.get_param("~levels", 1)
        px = rospy.get_param("~x", 0.0)
        name = rospy.get_param("~name", "ball")

        body = SoftBody()
        body.name = name
        body.pose.position.x = px
        body.pose.position.z = radius
        body.pose.orientation.w = 1.0

        # from http://blog.andreaskahler.com/2009/06/creating-icosphere-mesh-in-code.html
        icost = (1.0 + math.sqrt(5.0)) / 2.0
        tmp_radius = math.sqrt(1.0 + icost * icost)
        scale = radius / tmp_radius

        pt = Point()
        ico = []
        ico.append(Point(-1, icost, 0))
        ico.append(Point(1, icost, 0))
        ico.append(Point(-1, -icost, 0))
        ico.append(Point(1, -icost, 0))

        ico.append(Point(0, -1, icost))
        ico.append(Point(0, 1, icost))
        ico.append(Point(0, -1, -icost))
        ico.append(Point(0, 1, -icost))

        ico.append(Point(icost, 0, -1))
        ico.append(Point(icost, 0, 1))
        ico.append(Point(-icost, 0, -1))
        ico.append(Point(-icost, 0, 1))

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
            ico, faces = self.subdivide(ico, faces)

        print len(ico)
        # scale all points to be on sphere of desired radius
        for i in range(len(ico)):
            pt = ico[i]
            scale = radius / math.sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z)
            pt.x *= scale
            pt.y *= scale
            pt.z *= scale
            ico[i] = pt

        if True:
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
                marker.points.append(ico[face[0]])
                marker.points.append(ico[face[1]])
                marker.points.append(ico[face[2]])

            rospy.sleep(0.5)
            self.pub.publish(marker)
            rospy.sleep(0.5)

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

    def mid_point(self, v1, v2):
        pt = Point(
                   (v1.x + v2.x) / 2.0,
                   (v1.y + v2.y) / 2.0,
                   (v1.z + v2.z) / 2.0)
        # make all the points have the right radius
        if False:
            scale = self.tmp_radius / math.sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z)
            pt.x *= scale
            pt.y *= scale
            pt.z *= scale
        return pt

    def subdivide(self, pts, faces):  # v1, v2, v3, levels=1):
        #        v2
        #
        #     a     b
        #  
        #   v1   c    v3
        subfaces = []
        subpts = copy.deepcopy(pts)
        for fc in faces:
            subpts.append(self.mid_point(pts[fc[0]], pts[fc[1]]))
            ai = len(subpts) - 1
            subpts.append(self.mid_point(pts[fc[1]], pts[fc[2]]))
            bi = len(subpts) - 1
            subpts.append(self.mid_point(pts[fc[2]], pts[fc[0]]))
            ci = len(subpts) - 1

            subfaces.extend([[fc[0], ai, ci],
                             [ci, ai, bi],
                             [ci, bi, fc[2]],
                             [ai, fc[1], bi],
                             ])
        return (subpts, subfaces)

if __name__ == '__main__':
    rospy.init_node('icosphere')
    icosphere = Icosphere()

