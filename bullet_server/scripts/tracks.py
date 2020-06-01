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

        obstacle = Body()
        obstacle.name = "obstacle_1"
        obstacle.type = Body.BOX
        obstacle.mass = 0.0
        obstacle.pose.orientation.w = 1.0
        obstacle.pose.position.x = -2.0
        obstacle.scale.x = 1.0
        obstacle.scale.y = 1.0
        obstacle.scale.z = 0.07
        add_compound_request.body.append(obstacle)

        num_tracks = rospy.get_param("~num_tracks", 25)
        track_width = rospy.get_param("~track_width", 0.1)
        track_length = rospy.get_param("~track_length", 0.04)
        track_height = rospy.get_param("~track_height", 0.02)
        track_mass = rospy.get_param("~track_mass", 0.1)

        gap = rospy.get_param("~gap", 0.04)
        # spawn in a circle for now
        radius = (track_width + gap) * num_tracks / (2.0 * math.pi)

        track = Body()
        track.type = Body.BOX
        track.mass = track_mass
        track.scale.x = track_length
        track.scale.y = track_width
        track.scale.z = track_height

        for k in range(2):

            add_track = True
            if add_track:
                for i in range(num_tracks):
                    trk = copy.deepcopy(track)
                    trk.name = "track_" + str(k) + "_" + str(i)
                    fr = float(i) / float(num_tracks)
                    angle = 2.0 * math.pi * fr
                    rot = tf.transformations.quaternion_from_euler(0, -angle + math.pi/2.0, 0)
                    trk.pose.orientation.x = rot[0]
                    trk.pose.orientation.y = rot[1]
                    trk.pose.orientation.z = rot[2]
                    trk.pose.orientation.w = rot[3]

                    trk.pose.position.x = 1.5 * radius * math.cos(angle)
                    trk.pose.position.y = -0.5 + k * 1.0
                    trk.pose.position.z = 1.2 * radius + 0.75 * radius * math.sin(angle)
                    add_compound_request.body.append(trk)

                # add hinges between tracks
                for i in range(num_tracks):
                    constraint = Constraint()
                    constraint.name = "hinge_" + str(k) + "_" + str(i)
                    constraint.body_a = "track_" + str(k) + "_" + str(i)
                    constraint.body_b = "track_" + str(k) + "_" + str((i + 1) % num_tracks)
                    constraint.type = Constraint.HINGE
                    constraint.pivot_in_a.x = -(track_length / 2.0 + gap)
                    constraint.pivot_in_b.x = (track_length / 2.0 + gap)
                    constraint.axis_in_a.y = 1.0
                    constraint.axis_in_b.y = 1.0
                    constraint.enable_pos_pub = False
                    constraint.enable_motor_sub = False
                    add_compound_request.constraint.append(constraint)

            wheel_spacing = rospy.get_param("~wheel_spacing", 1.1)
            chassis = Body()
            chassis.name = "chassis"
            chassis.type = Body.BOX
            chassis.mass = 2.0
            chassis.pose.orientation.w = 1.0
            chassis.scale.x = wheel_spacing * 0.55
            chassis.scale.y = 0.3
            chassis.scale.z = 0.07
            chassis.pose.position.z = radius
            add_compound_request.body.append(chassis)

            tracks_per_circumference = 10
            wheel_circumference = (track_length * 2.0 + gap) * tracks_per_circumference
            wheel_radius = wheel_circumference / (2.0 * math.pi)
            for i in range(2):
                wheel = Body()
                wheel.type = Body.CYLINDER
                wheel.name = "wheel_" + str(k) + "_" + str(i)
                wheel.mass = 1.0
                wheel.pose.orientation.w = 1.0
                wheel.scale.x = wheel_radius
                wheel.scale.y = track_width * 1.1
                wheel.scale.z = wheel_radius
                wheel.pose.position.x = -wheel_spacing / 2.0 + i * wheel_spacing
                wheel.pose.position.y = -0.5 + 1.0 * k
                wheel.pose.position.z = radius
                add_compound_request.body.append(wheel)

                for j in range(tracks_per_circumference):
                    fr = float(j) / float(tracks_per_circumference)
                    angle = fr * 2.0 * math.pi
                    tooth = Body()
                    tooth.name = "tooth_" + str(k) + "_" + str(i) + "_" + str(j)
                    tooth.type = Body.BOX
                    tooth.mass = 0.1
                    tooth_angle = angle
                    rot = tf.transformations.quaternion_from_euler(0, tooth_angle - math.pi / 2.0, 0)
                    tooth.pose.orientation.x = rot[0]
                    tooth.pose.orientation.y = rot[1]
                    tooth.pose.orientation.z = rot[2]
                    tooth.pose.orientation.w = rot[3]
                    tooth.scale.x = gap * 0.8
                    tooth.scale.y = track_width
                    tooth.scale.z = gap * 0.35
                    x = wheel_radius * math.cos(angle)
                    z = wheel_radius * math.sin(angle)
                    tooth.pose.position.x = wheel.pose.position.x + x
                    tooth.pose.position.y = wheel.pose.position.y
                    tooth.pose.position.z = wheel.pose.position.z + z
                    add_compound_request.body.append(tooth)

                    if True:
                        fixed = Constraint()
                        fixed.name = "attach_" + tooth.name
                        fixed.body_a = wheel.name
                        fixed.body_b = tooth.name
                        fixed.type = Constraint.HINGE
                        # TODO(lucasw) this doesn't match what rot does above- as the sim
                        # start the tooth rotates to this constraint
                        fixed.lower_ang_lim = tooth_angle - 0.01
                        fixed.upper_ang_lim = tooth_angle + 0.01
                        # TODO(lucasw) Something is broken with fixed
                        # fixed.type = Constraint.FIXED
                        fixed.pivot_in_a.x = x
                        fixed.pivot_in_a.y = 0
                        fixed.pivot_in_a.z = z
                        fixed.axis_in_a.y = 1.0
                        fixed.axis_in_b.y = 1.0
                        fixed.enable_pos_pub = False
                        fixed.enable_motor_sub = False
                        add_compound_request.constraint.append(fixed)

                axle = Constraint()
                axle.name = "wheel_motor_" + str(k) + "_" + str(i)
                axle.body_a = "chassis"
                axle.body_b = wheel.name
                axle.type = Constraint.HINGE
                axle.lower_ang_lim = -4.0
                axle.upper_ang_lim = 4.0
                axle.max_motor_impulse = 25000.0
                axle.pivot_in_a.x = wheel.pose.position.x
                axle.pivot_in_a.y = -0.5 + 1.0 * k
                axle.pivot_in_b.y = 0.0
                axle.axis_in_a.y = 1.0
                axle.axis_in_b.y = 1.0
                axle.enable_pos_pub = True
                axle.enable_motor_sub = True
                add_compound_request.constraint.append(axle)

                # try to constraint the track- could also try two cones
                cover_offset = track_width * 1.15
                # Outer cover
                cover = Body()
                cover.type = Body.CYLINDER
                cover.name = "outer_cover_" + str(k) + "_" + str(i)
                cover.mass = 1.0
                cover.pose.orientation.w = 1.0
                cover.scale.x = wheel_radius * 1.1
                cover.scale.y = track_width * 0.1
                cover.scale.z = wheel_radius * 1.1
                cover.pose.position.x = -wheel_spacing / 2.0 + i * wheel_spacing
                cover.pose.position.y = -0.5 - cover_offset + (1.0 + 2.0 * cover_offset) * k
                cover.pose.position.z = radius
                add_compound_request.body.append(cover)

                axle = Constraint()
                axle.name = "inner_cover_constraint_" + str(k) + "_" + str(i)
                axle.body_a = wheel.name
                axle.body_b = cover.name
                axle.type = Constraint.HINGE
                axle.lower_ang_lim = -0.01
                axle.upper_ang_lim = 0.01
                axle.max_motor_impulse = 0.0
                axle.pivot_in_a.x = 0.0
                axle.pivot_in_a.y = cover.pose.position.y - wheel.pose.position.y
                axle.pivot_in_b.y = 0.0
                axle.axis_in_a.y = 1.0
                axle.axis_in_b.y = 1.0
                axle.enable_pos_pub = False
                axle.enable_motor_sub = False
                add_compound_request.constraint.append(axle)

                # Inner cover
                cover = Body()
                cover.type = Body.CYLINDER
                cover.name = "inner_cover_" + str(k) + "_" + str(i)
                cover.mass = 1.0
                cover.pose.orientation.w = 1.0
                cover.scale.x = wheel_radius * 1.1
                cover.scale.y = track_width * 0.1
                cover.scale.z = wheel_radius * 1.1
                cover.pose.position.x = -wheel_spacing / 2.0 + i * wheel_spacing
                cover.pose.position.y = -0.5 + cover_offset + (1.0 - 2.0 * cover_offset) * k
                cover.pose.position.z = radius
                add_compound_request.body.append(cover)

                axle = Constraint()
                axle.name = "outer_cover_constraint_" + str(k) + "_" + str(i)
                axle.body_a = wheel.name
                axle.body_b = cover.name
                axle.type = Constraint.HINGE
                axle.lower_ang_lim = -0.01
                axle.upper_ang_lim = 0.01
                axle.max_motor_impulse = 0.0
                axle.pivot_in_a.x = 0.0
                axle.pivot_in_a.y = cover.pose.position.y - wheel.pose.position.y
                axle.pivot_in_b.y = 0.0
                axle.axis_in_a.y = 1.0
                axle.axis_in_b.y = 1.0
                axle.enable_pos_pub = False
                axle.enable_motor_sub = False
                add_compound_request.constraint.append(axle)

        # print add_compound_request
        try:
            add_compound_response = self.add_compound(add_compound_request)
            rospy.loginfo(add_compound_response)
        except rospy.service.ServiceException as e:
            rospy.logerr(e)


if __name__ == '__main__':
    rospy.init_node('tracked_vehicle')
    tracked_vehicle = TrackedVehicle()
