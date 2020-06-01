#!/usr/bin/env python
# Copyright (c) 2016 Lucas Walter
# November 2016

# create a ground object

import rospy
import tf
import tf2_ros

from bullet_server.msg import Body, Face, Link, Material, Node, SoftBody, SoftConfig, Tetra
from bullet_server.srv import *
from geometry_msgs.msg import TransformStamped
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *


class GroundPlane:
    def __init__(self):
        rospy.wait_for_service('add_compound')
        self.add_compound = rospy.ServiceProxy('add_compound', AddCompound)

        add_compound_request = AddCompoundRequest()
        add_compound_request.remove = False
        # make the top cylinder plate
        ground = Body()
        ground.name = "ground"
        ground.mass = 0.0
        rot90 = tf.transformations.quaternion_from_euler(0, 0, 0)
        radius = 50
        thickness = 1.0
        ground.pose.orientation.x = rot90[0]
        ground.pose.orientation.y = rot90[1]
        ground.pose.orientation.z = rot90[2]
        ground.pose.orientation.w = rot90[3]
        ground.pose.position.z = -thickness
        ground.type = Body.BOX
        ground.scale.x = radius
        ground.scale.y = radius
        ground.scale.z = thickness
        ground.friction = 0.7
        add_compound_request.body.append(ground)

        try:
            add_compound_response = self.add_compound(add_compound_request)
            rospy.loginfo(add_compound_response)
        except rospy.service.ServiceException as e:
            rospy.logerr(e)


if __name__ == '__main__':
    rospy.init_node('ground_plane')
    ground_plane = GroundPlane()
