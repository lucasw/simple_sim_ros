#!/usr/bin/env python
# Copyright 2016 Lucas Walter
# 
# create a number of bodies and joints in the bullet server

import random
import rospkg
import rospy

from bullet_server.msg import Body, Constraint, Heightfield, Impulse


rospy.init_node("random")

body_pub = rospy.Publisher("add_body", Body, queue_size=5)
constraint_pub = rospy.Publisher("add_constraint", Constraint, queue_size=1)
impulse_pub = rospy.Publisher("add_impulse", Impulse, queue_size=1)

sleep_time = 2.0
postfix = str(random.randrange(0, 1000000000))
count = 0
while not rospy.is_shutdown():
    z_height = 8.8
    wheel = Body()
    wheel.name = "random_body_" + postfix + "_" + str(count)
    wheel.type = random.randint(0, 4)  # Body.CYLINDER
    wheel.pose.position.x = random.uniform(-0.1, 0.1)
    wheel.pose.position.y = random.uniform(-0.1, 0.1)
    wheel.pose.position.z = z_height + random.uniform(0, 2.0)
    # TODO(lucasw) make a random rotation
    wheel.pose.orientation.x = 0.0  # 0.707
    wheel.pose.orientation.w = random.uniform(0.9, 1.0)  # 0.707
    wheel.scale.x = random.uniform(0.5, 2.4)
    wheel.scale.y = random.uniform(0.5, 2.4)
    wheel.scale.z = random.uniform(0.5, 2.4)
    # republish wheel because usually the first one isn't received
    body_pub.publish(wheel)
    rospy.sleep(sleep_time)
    count += 1

    # TODO(lucasw) create random joints between existing links
    # store the names of all the existing bodies

