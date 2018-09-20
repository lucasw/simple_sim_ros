#!/usr/bin/env python
# Copyright 2016 Lucas Walter
#
# create a number of bodies and joints in the bullet server

import copy
import random
import rospkg
import rospy

from bullet_server.msg import Body, Constraint, Heightfield, Impulse
from bullet_server.srv import *

rospy.init_node("random")

pub = rospy.Publisher("add_impulse", Impulse, queue_size=3)
rospy.loginfo('waiting for services...')
rospy.wait_for_service('add_compound')
rospy.loginfo('service found')
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
    body2 = copy.deepcopy(body)
    body2.name += "_non_kinematic"
    body2.mass = 1.0
    body2.kinematic = False
    body2.pose.position.z = 2.0
    add_compound_request.body.append(body2)
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
    impulse.impulse.x = 0.15
    pub.publish(impulse)

    set_transform = rospy.ServiceProxy('set_transform', SetTransform)
    while not rospy.is_shutdown():
        rospy.sleep(1.5)

        set_transform_request = SetTransformRequest()
        set_transform_request.body = body.name
        set_transform_request.transform.rotation.w = 1.0
        set_transform_request.transform.translation.x = -2.0
        set_transform_request.transform.translation.y = random.random()
        try:
            set_transform_response = set_transform(set_transform_request)
            rospy.loginfo(set_transform_response)
        except rospy.service.ServiceException as e:
            rospy.logerr(e)
        set_transform_request = SetTransformRequest()
        set_transform_request.body = body2.name
        set_transform_request.transform.rotation.w = 1.0
        set_transform_request.transform.translation.x = 0.0
        set_transform_request.transform.translation.y = random.random()
        set_transform_request.transform.translation.z = 2.0
        try:
            set_transform_response = set_transform(set_transform_request)
            rospy.loginfo(set_transform_response)
        except rospy.service.ServiceException as e:
            rospy.logerr(e)

rospy.spin()
