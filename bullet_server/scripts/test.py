#!/usr/bin/env python
# Copyright 2016 Lucas Walter
#
# create a number of bodies and joints in the bullet server

import rospkg
import rospy

from bullet_server.msg import Body, Constraint, Heightfield, Impulse


rospy.init_node("test_bullet_server")

body_pub = rospy.Publisher("add_body", Body, queue_size=5)
constraint_pub = rospy.Publisher("add_constraint", Constraint, queue_size=1)
impulse_pub = rospy.Publisher("add_impulse", Impulse, queue_size=1)

rospy.sleep(1.0)

sleep_time = 0.1

# Car
z_height = 8.8
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
# republish wheel because usually the first one isn't received
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

# exit()

axle_y = 0.75
axle = Constraint()
axle.name = "axle0"
axle.type = Constraint.POINT2POINT
axle.body_a = "chassis0"
axle.body_b = "wheel0"
axle.pivot_in_a.x = 1.0  # wheel0.pose.position.x
axle.pivot_in_a.y = axle_y
axle.pivot_in_a.z = -0.1
axle.pivot_in_b.y = -0.2
constraint_pub.publish(axle)
rospy.sleep(sleep_time)

axle.name = "axle1"
axle.body_b = "wheel1"
axle.pivot_in_a.y = -axle_y
axle.pivot_in_b.y = 0.2
constraint_pub.publish(axle)
rospy.sleep(sleep_time)

axle.name = "axle2"
axle.body_b = "wheel2"
axle.pivot_in_a.x = -1.0
constraint_pub.publish(axle)
rospy.sleep(sleep_time)

axle.name = "axle3"
axle.body_b = "wheel3"
axle.pivot_in_a.y = axle_y
axle.pivot_in_b.y = -0.2
constraint_pub.publish(axle)
rospy.sleep(sleep_time)

if False:
    impulse = Impulse()
    impulse.body = "chassis0"
    impulse.impulse.x = -1.4

    while not rospy.is_shutdown():
        impulse_pub.publish(impulse)
        rospy.sleep(1.0)


# rospy.spin()
