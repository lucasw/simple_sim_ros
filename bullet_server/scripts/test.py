#!/usr/bin/env python
# Copyright 2016 Lucas Walter
# 
# create a number of bodies and joints in the bullet server

import rospy

from bullet_server.msg import Body, Constraint, Impulse


rospy.init_node("test_bullet_server")

body_pub = rospy.Publisher("add_body", Body, queue_size=5)
constraint_pub = rospy.Publisher("add_constraint", Constraint, queue_size=1)
impulse_pub = rospy.Publisher("add_impulse", Impulse, queue_size=1)

rospy.sleep(1.0)

sleep_time = 0.1

wheel = Body()
wheel.name = "wheel0"
wheel.type = Body.CYLINDER
wheel.pose.position.x = 1.0
wheel.pose.position.y = 1.0
wheel.pose.position.z = 1.0
wheel.pose.orientation.x = 0.0  # 0.707
wheel.pose.orientation.w = 1.0  # 0.707
wheel.scale.x = 0.5
wheel.scale.y = 0.3
body_pub.publish(wheel)
rospy.sleep(sleep_time)
body_pub.publish(wheel)
rospy.sleep(sleep_time)

wheel.name = "wheel1"
wheel.pose.position.y = -1.0
body_pub.publish(wheel)
rospy.sleep(sleep_time)

wheel.name = "wheel2"
wheel.pose.position.x = -1.0
body_pub.publish(wheel)
rospy.sleep(sleep_time)

wheel.name = "wheel3"
wheel.pose.position.y = 1.0
body_pub.publish(wheel)
rospy.sleep(sleep_time)

#chassis = Body()

rospy.spin()
