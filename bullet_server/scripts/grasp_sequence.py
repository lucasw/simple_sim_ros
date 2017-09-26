#!/usr/bin/env python

import rospy

from std_msgs.msg import Float64


class GraspSequence:
    def __init__(self):
        self.pubs = {}

        self.pubs['arm'] = rospy.Publisher("prismatic_upper_fore/setpoint", Float64, queue_size=4)
        num_fingers = 3
        for i in range(num_fingers):
            topic = "finger_joint_" + str(i) + "/setpoint"
            self.pubs['finger_joint_' + str(i)] = rospy.Publisher(topic, Float64, queue_size=4)
            topic = "finger_joint_" + str(i) + "/setpoint"
            self.pubs['finger_lower_joint_' + str(i)] = rospy.Publisher(topic, Float64, queue_size=4)

        rospy.sleep(0.5)

        fing_open = -0.4
        self.setpoint(0.0, fing_open, fing_open, fing_open, fing_open, fing_open, fing_open)
        rospy.sleep(2.0)
        self.setpoint(0.0, fing_open, fing_open, fing_open, fing_open, fing_open, fing_open)
        rospy.sleep(2.0)
        self.setpoint(0.1, fing_open, fing_open, fing_open, fing_open, fing_open, fing_open)
        rospy.sleep(2.0)
        fing_closed = -0.05
        self.setpoint(0.1, fing_closed, fing_closed, fing_closed, fing_closed, fing_closed, fing_closed)
        rospy.sleep(2.0)
        # rospy.spin()

    def setpoint(self, arm, f0, f1, f2, fu0, fu1, fu2):
        self.pubs['arm'].publish(Float64(arm))
        self.pubs['finger_joint_0'].publish(Float64(f0))
        self.pubs['finger_joint_1'].publish(Float64(f1))
        self.pubs['finger_joint_2'].publish(Float64(f2))
        self.pubs['finger_lower_joint_0'].publish(Float64(fu0))
        self.pubs['finger_lower_joint_1'].publish(Float64(fu1))
        self.pubs['finger_lower_joint_2'].publish(Float64(fu2))

if __name__ == '__main__':
    rospy.init_node('grasp_sequence')
    grasp_sequence = GraspSequence()

