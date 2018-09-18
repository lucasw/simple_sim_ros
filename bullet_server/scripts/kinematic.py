#!/usr/bin/env python
# Copyright 2016 Lucas Walter
#
# create a number of bodies and joints in the bullet server

import random
import rospkg
import rospy

from bullet_server.msg import Body, Constraint, Heightfield, Impulse
from bullet_server.srv import *

rospy.init_node("random")

pub = rospy.Publisher("add_impulse", Impulse, queue_size=3)
rospy.wait_for_service('add_compound')
add_compound = rospy.ServiceProxy('add_compound', AddCompound)

# body_pub = rospy.Publisher("add_body", Body, queue_size=5)
# constraint_pub = rospy.Publisher("add_constraint", Constraint, queue_size=1)
# impulse_pub = rospy.Publisher("add_impulse", Impulse, queue_size=1)

sleep_time = 2.0
count = 0

bodies = {}

z_height = rospy.get_param("~z", 0.8)

if True:
    body = Body()
    body.name = "kinematic_body"
    bodies[count] = body.name
    body.type = random.randint(0, 4)  # Body.CYLINDER
    body.kinematic = True
    body.mass = 0.0

    body.pose.position.x = random.uniform(-0.1, 0.1)
    body.pose.position.y = random.uniform(-0.1, 0.1)
    body.pose.position.z = z_height  # + random.uniform(0, 4.0)
    # TODO(lucasw) make a random rotation
    body.pose.orientation.x = 0.0  # 0.707
    body.pose.orientation.w = random.uniform(0.9, 1.0)  # 0.707
    body.scale.x = random.uniform(0.1, 0.4)
    body.scale.y = random.uniform(0.1, 0.4)
    body.scale.z = random.uniform(0.1, 0.4)
    # republish body because usually the first one isn't received
    add_compound_request = AddCompoundRequest()
    add_compound_request.remove = rospy.get_param('~remove', False)
    add_compound_request.body.append(body)
    try:
        add_compound_response = add_compound(add_compound_request)
        rospy.loginfo(add_compound_response)
    except rospy.service.ServiceException as e:
        rospy.logerr(e)

    rospy.sleep(sleep_time)
    count += 1

    # TODO(lucasw) create random joints between existing links
    # store the names of all the existing bodies
    min_count = 8

    rospy.sleep(1.0)
    impulse = Impulse()
    impulse.body = body.name
    impulse.impulse.x = 0.05
    pub.publish(impulse)
rospy.spin()
