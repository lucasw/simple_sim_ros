#!/usr/bin/env python
# Copyright 2016 Lucas Walter
#
# create a number of bodies and joints in the bullet server

import cv2
import cv_bridge
import rospkg
import rospy

from bullet_server.msg import Body, Constraint, Heightfield, Impulse


rospy.init_node("test_bullet_server")

heightfield_pub = rospy.Publisher("add_heightfield", Heightfield, queue_size=5)

rospy.sleep(1.0)

sleep_time = 0.1

# Heightfield
bridge = cv_bridge.CvBridge()
rospack = rospkg.RosPack()
heightfield = Heightfield()
heightfield.name = "jpg_test"
# name = "heightfield_small2.jpg"
name = "heightfield.jpg"
image = cv2.imread(rospack.get_path('bullet_server') + "/data/" + name, 0)
# cv2.imshow("image", image)
# cv2.waitKey(0)
heightfield.image = bridge.cv2_to_imgmsg(image, encoding="mono8")
heightfield.resolution = 64.0 / image.shape[0]
heightfield.height_scale = 6.0 / 255.0
heightfield.image.header.frame_id = "map"
heightfield_pub.publish(heightfield)

rospy.sleep(sleep_time * 9)
