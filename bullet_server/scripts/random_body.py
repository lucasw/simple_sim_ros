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

bodies = {}

while not rospy.is_shutdown():
    z_height = 8.8
    wheel = Body()
    wheel.name = "random_body_" + postfix + "_" + str(count)
    bodies[count] = wheel.name
    wheel.type = random.randint(0, 4)  # Body.CYLINDER
    wheel.pose.position.x = random.uniform(-0.1, 0.1)
    wheel.pose.position.y = random.uniform(-0.1, 0.1)
    wheel.pose.position.z = z_height + random.uniform(0, 4.0)
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
    min_count = 8
    if count > min_count and random.random() > 0.5:
        axel = Constraint()
        axel.name = "constraint_" + postfix + "_" + str(count)
        axel.type = Constraint.POINT2POINT
        axel.body_a = bodies[random.randrange(count - min_count, count)]
        axel.body_b = bodies[random.randrange(count - min_count, count)]
        if (axel.body_a == axel.body_b):
            continue
        axel.pivot_in_a.x = random.uniform(2.5, 3.0)
        axel.pivot_in_a.y = random.uniform(-3.0, 3.0)
        axel.pivot_in_a.z = random.uniform(-3.0, 3.0)
        axel.pivot_in_b.x = random.uniform(-2.5, -3.0)
        axel.pivot_in_b.y = random.uniform(-3.0, 3.0)
        axel.pivot_in_b.z = random.uniform(-3.0, 3.0)
        constraint_pub.publish(axel)
        rospy.loginfo(axel)
        rospy.sleep(sleep_time)


