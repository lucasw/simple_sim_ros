#!/usr/bin/env python
# Copyright (c) 2016 Lucas Walter
# Make a tank-like tracked vehicle
# This may require some 'cheats' to work and be computationally modest.

# First pass:
# Create a bunch of flat rigid body plates, connect them in a loop
# with single axis rotation joints.
# Have a drive wheel and roller wheels inside the loop.
# Perhaps put ridges on the drive wheel and gaps between the tread plates,
# and see if the tracks stay on.
# Then add additional rigid bodies to try to keep the tracks on and
# allow them to be propelled forward.

import copy
import math
import rospy
import tf

from bullet_server.msg import Body, Constraint, Heightfield, Impulse
from bullet_server.srv import *


class TrackedVehicle:
    def __init__(self):
        rospy.wait_for_service('add_compound')
        self.add_compound = rospy.ServiceProxy('add_compound', AddCompound)
        add_compound_request = AddCompoundRequest()
        add_compound_request.remove = rospy.get_param('~remove', False)

	num_tracks = rospy.get_param("~num_tracks", 30)
	track_width = rospy.get_param("~track_width", 0.1)
	track_length = rospy.get_param("~track_width", 0.04)
	track_height = rospy.get_param("~track_height", 0.01)
	track_mass = rospy.get_param("~track_mass", 0.1)

        gap = rospy.get_param("~gap", 0.03)
        # spawn in a circle for now
        radius = (track_width + gap) * num_tracks / (2.0 * math.pi)

	track = Body()
        track.type = Body.BOX
	track.mass = track_mass
	track.scale.x = track_length
	track.scale.y = track_width
        track.scale.z = track_height

        for i in range(num_tracks):
            trk = copy.deepcopy(track)
            trk.name = "track_" + str(i)
            fr = float(i) / float(num_tracks)
            angle = 2.0 * math.pi * fr
            rot = tf.transformations.quaternion_from_euler(0, -angle + math.pi/2.0, 0)
            trk.pose.orientation.x = rot[0]
            trk.pose.orientation.y = rot[1]
            trk.pose.orientation.z = rot[2]
            trk.pose.orientation.w = rot[3]

	    trk.pose.position.x = radius * math.cos(angle)
            trk.pose.position.y = -0.5
            trk.pose.position.z = 1.2 * radius + radius * math.sin(angle)
            add_compound_request.body.append(trk)

        # add hinges between tracks
        for i in range(num_tracks):
            constraint = Constraint()
            constraint.name = "hinge_" + str(i)
            constraint.body_a = "track_" + str(i)
            constraint.body_b = "track_" + str((i + 1) % num_tracks)
            constraint.type = Constraint.HINGE
            constraint.pivot_in_a.x = -(track_length/2.0 + gap)
            constraint.pivot_in_b.x = (track_length/2.0 + gap)
            constraint.axis_in_a.y = 1.0
            constraint.axis_in_b.y = 1.0
            add_compound_request.constraint.append(constraint)

        chassis = Body()
        chassis.name = "chassis"
        chassis.type = Body.BOX
        chassis.mass = 1.0
        chassis.pose.orientation.w = 1.0
        chassis.scale.x = 1.0
        chassis.scale.y = 0.38
        chassis.scale.z = 0.07
        chassis.pose.position.z = radius
        add_compound_request.body.append(chassis)

	wheel = Body()
        wheel.type = Body.CYLINDER
        wheel.name = "wheel"
	wheel.mass = 1.0
        wheel.pose.orientation.w = 1.0
	wheel.scale.x = 0.2
	wheel.scale.y = track_width
        wheel.scale.z = 0.2
        wheel.pose.position.y = -0.5
        wheel.pose.position.z = radius
        add_compound_request.body.append(wheel)

        axle = Constraint()
        axle.name = "wheel_motor"
        axle.body_a = "chassis"
        axle.body_b = "wheel"
        axle.type = Constraint.HINGE
        axle.lower_ang_lim = -4.0
        axle.upper_ang_lim = 4.0
        axle.max_motor_impulse = 25000.0
        axle.pivot_in_a.y = -0.5
        axle.pivot_in_b.y = 0.0
        axle.axis_in_a.y = 1.0
        axle.axis_in_b.y = 1.0
        add_compound_request.constraint.append(axle)

        try:
            add_compound_response = self.add_compound(add_compound_request)
            rospy.loginfo(add_compound_response)
        except rospy.service.ServiceException as e:
            rospy.logerr(e)

if __name__ == '__main__':
    rospy.init_node('tracked_vehicle')
    tracked_vehicle = TrackedVehicle()
