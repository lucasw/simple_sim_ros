#!/usr/bin/env python
# generate an icosphere soft body

import math
import rospy

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


class Icosphere:
    def __init__(self):
        self.pub = rospy.Publisher("marker", Marker, queue_size=2)

        marker = Marker()
        marker.header.frame_id = rospy.get_param("~frame_id", "map")
        marker.ns = marker.header.frame_id
        marker.id = 1
        # marker.type = Marker.LINE_LIST   # POINTS  # TRIANGLE_LIST
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        if False:
            marker.scale.x = 0.1  # 1.0
            marker.scale.y = 0.1  # 1.0
            marker.scale.z = 0.1  # 1.0
        else:
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0

        marker.color.a = 1.0
        marker.color.r = rospy.get_param("~r", 1.0)
        marker.color.g = rospy.get_param("~g", 1.0)
        marker.color.b = rospy.get_param("~b", 1.0)

        radius = rospy.get_param("~radius", 2.0)

        # from http://blog.andreaskahler.com/2009/06/creating-icosphere-mesh-in-code.html
        icost = (1.0 + math.sqrt(5.0)) / 2.0

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
        rospy.loginfo(ico)

        self.make_triangle(ico, marker.points, 0, 11, 5)
        self.make_triangle(ico, marker.points, 0, 5, 1)
        self.make_triangle(ico, marker.points, 0, 1, 7)
        self.make_triangle(ico, marker.points, 0, 7, 10)
        self.make_triangle(ico, marker.points, 0, 10, 11)

        self.make_triangle(ico, marker.points, 1, 5, 9)
        self.make_triangle(ico, marker.points, 5, 11, 4)
        self.make_triangle(ico, marker.points, 11, 10, 2)
        self.make_triangle(ico, marker.points, 10, 7, 6)
        self.make_triangle(ico, marker.points, 7, 1, 8)

        self.make_triangle(ico, marker.points, 3, 9, 4)
        self.make_triangle(ico, marker.points, 3, 4, 2)
        self.make_triangle(ico, marker.points, 3, 2, 6)
        self.make_triangle(ico, marker.points, 3, 6, 8)
        self.make_triangle(ico, marker.points, 3, 8, 9)

        self.make_triangle(ico, marker.points, 4, 9, 5)
        self.make_triangle(ico, marker.points, 2, 4, 11)
        self.make_triangle(ico, marker.points, 6, 2, 10)
        self.make_triangle(ico, marker.points, 8, 6, 7)
        self.make_triangle(ico, marker.points, 9, 8, 1)

        rospy.sleep(0.5)
        self.pub.publish(marker)
        rospy.sleep(0.5)

    def make_triangle(self, src_pts, dst_pts, i0, i1, i2):
        dst_pts.append(src_pts[i0])
        # dst_pts.append(src_pts[i1])

        dst_pts.append(src_pts[i1])
        # dst_pts.append(src_pts[i2])

        dst_pts.append(src_pts[i2])
        # dst_pts.append(src_pts[i0])

if __name__ == '__main__':
    rospy.init_node('icosphere')
    icosphere = Icosphere()

