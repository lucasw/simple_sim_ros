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
body_pub = rospy.Publisher("add_body", Body, queue_size=5)
constraint_pub = rospy.Publisher("add_constraint", Constraint, queue_size=1)
impulse_pub = rospy.Publisher("add_impulse", Impulse, queue_size=1)

rospy.sleep(1.0)

sleep_time = 0.1

# Car
z_height = 1.8
wheel = Body()
wheel.name = "wheel0"
wheel.type = Body.CYLINDER
wheel.pose.position.x = 1.0
wheel.pose.position.y = 1.0
wheel.pose.position.z = z_height
wheel.pose.orientation.x = 0.0  # 0.707
wheel.pose.orientation.w = 1.0  # 0.707
wheel.scale.x = 0.5
wheel.scale.y = 0.2
body_pub.publish(wheel)
rospy.sleep(sleep_time)
body_pub.publish(wheel)
rospy.sleep(sleep_time)


# Heightfield
bridge = cv_bridge.CvBridge()
rospack = rospkg.RosPack()
heightfield = Heightfield()
heightfield.name = "jpg_test"
image = cv2.imread(rospack.get_path('bullet_server') + "/data/heightfield_small.jpg", 0)
# cv2.imshow("image", image)
# cv2.waitKey(0)
heightfield.image = bridge.cv2_to_imgmsg(image, encoding="mono8")
heightfield.resolution = 16.0 / image.shape[0]
heightfield.height_scale = 2.0 / 255.0
heightfield.image.header.frame_id = "map"
heightfield_pub.publish(heightfield)
rospy.sleep(sleep_time * 5)

# back to car
wheel.name = "wheel1"
wheel.pose.position.y = -1.0
body_pub.publish(wheel)
rospy.sleep(sleep_time)

exit()

wheel.name = "wheel2"
wheel.pose.position.x = -1.0
body_pub.publish(wheel)
rospy.sleep(sleep_time)

wheel.name = "wheel3"
wheel.pose.position.y = 1.0
body_pub.publish(wheel)
rospy.sleep(sleep_time)

chassis = Body()
chassis.name = "chassis0"
chassis.type = Body.BOX
chassis.pose.position.x = 0.0
chassis.pose.position.y = 0.0
chassis.pose.position.z = z_height
chassis.pose.orientation.x = 0.0  # 0.707
chassis.pose.orientation.w = 1.0  # 0.707
chassis.scale.x = 1.2
chassis.scale.y = 0.5
chassis.scale.z = 0.25
body_pub.publish(chassis)
rospy.sleep(sleep_time)


axel = Constraint()
axel.name = "axel0"
axel.type = Constraint.POINT2POINT
axel.body_a = "chassis0"
axel.body_b = "wheel0"
axel.pivot_in_a.x = 1.0  # wheel0.pose.position.x
axel.pivot_in_a.y = 0.6
axel.pivot_in_a.z = -0.1
axel.pivot_in_b.y = -0.2
constraint_pub.publish(axel)
rospy.sleep(sleep_time)

axel.name = "axel1"
axel.body_b = "wheel1"
axel.pivot_in_a.y = -0.75
axel.pivot_in_b.y = 0.2
constraint_pub.publish(axel)
rospy.sleep(sleep_time)

axel.name = "axel2"
axel.body_b = "wheel2"
axel.pivot_in_a.x = -1.0
constraint_pub.publish(axel)
rospy.sleep(sleep_time)

axel.name = "axel3"
axel.body_b = "wheel3"
axel.pivot_in_a.y = 0.75
axel.pivot_in_b.y = -0.2
constraint_pub.publish(axel)
rospy.sleep(sleep_time)

if False:
    impulse = Impulse()
    impulse.body = "chassis0"
    impulse.impulse.x = 1

    while not rospy.is_shutdown():
        impulse_pub.publish(impulse)
        rospy.sleep(1.0)


# rospy.spin()
