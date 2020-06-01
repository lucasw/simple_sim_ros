#!/usr/bin/env python

import rospy
import sys

from std_msgs.msg import Float64


class GraspSequence:
    def __init__(self):
        self.pubs = {}

        self.pubs['arm'] = rospy.Publisher("prismatic_upper_fore/setpoint", Float64, queue_size=4)
        num_fingers = 3
        for i in range(num_fingers):
            prefix = "finger_joint_" + str(i)
            topic = prefix + "/setpoint"
            self.pubs[prefix] = rospy.Publisher(topic, Float64, queue_size=4)
            # rospy.set_param(topic + "/pid/Kp_scale", 10.0)
            prefix = "finger_lower_joint_" + str(i)
            topic = prefix + "/setpoint"
            self.pubs[prefix] = rospy.Publisher(topic, Float64, queue_size=4)
            # rospy.set_param(topic + "/pid/Kp_scale", 10.0)

        self.scale = rospy.get_param("~scale", 1.0)
        rospy.loginfo(self.pubs.keys())
        rospy.sleep(0.5)

        while not rospy.is_shutdown():
            self.sequence()

    def sequence(self):
        fing_open = -0.6
        self.setpoint(0.0, fing_open, fing_open, fing_open,
                      fing_open * 0.7, fing_open * 0.7, fing_open * 0.7)
        rospy.sleep(0.5)
        self.setpoint(0.0, fing_open, fing_open, fing_open,
                      fing_open * 0.7, fing_open * 0.7, fing_open * 0.7)
        rospy.sleep(0.5)
        grab_height = 0.229
        self.setpoint(grab_height * self.scale, fing_open, fing_open, fing_open,
                      fing_open * 0.7, fing_open * 0.7, fing_open * 0.7)
        rospy.sleep(3.0)
        fing_closed = -0.15
        lower_fing_closed = 0.9
        self.setpoint(grab_height * self.scale, fing_closed, fing_closed, fing_closed,
                      lower_fing_closed, lower_fing_closed, lower_fing_closed)
        rospy.sleep(1.0)
        self.setpoint(0.0, fing_closed, fing_closed, fing_closed,
                      lower_fing_closed, lower_fing_closed, lower_fing_closed)
        rospy.sleep(3.0)
        # rospy.spin()

    def setpoint(self, arm, f0, f1, f2, fu0, fu1, fu2):
        for i in range(4):
            self.pubs['arm'].publish(Float64(arm))
            self.pubs['finger_joint_0'].publish(Float64(f0))
            self.pubs['finger_joint_1'].publish(Float64(f1))
            self.pubs['finger_joint_2'].publish(Float64(f2))
            rospy.sleep(0.05)
            self.pubs['finger_lower_joint_0'].publish(Float64(fu0))
            self.pubs['finger_lower_joint_1'].publish(Float64(fu1))
            self.pubs['finger_lower_joint_2'].publish(Float64(fu2))
            rospy.sleep(0.05)
            if rospy.is_shutdown():
                sys.exit()


if __name__ == '__main__':
    rospy.init_node('grasp_sequence')
    grasp_sequence = GraspSequence()
