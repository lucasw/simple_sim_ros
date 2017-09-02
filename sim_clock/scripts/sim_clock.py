#!/usr/bin/env python
# generate a clock
# have a dr interface for adjusting real-time factor
# (going backwards also allowed though rest of ros probably
# won't like it)
# updates per second
# or single stepping

import rospy
# import threading
import time

from rosgraph_msgs.msg import Clock


class SimClock():
    def __init__(self):
        self.start_time = 0.0
        self.cur_time = self.start_time
        self.dt = 0.01
        self.pub = rospy.Publisher("clock", Clock, queue_size=3)
        while not rospy.is_shutdown():
            msg = rospy.Time.from_sec(self.cur_time)
            self.pub.publish(msg)
            time.sleep(self.dt)
            self.cur_time += self.dt

if __name__ == '__main__':
    rospy.init_node('sim_clock')
    sim_clock = SimClock()
    # rospy.spin()

