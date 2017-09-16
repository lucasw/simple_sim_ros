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

from dynamic_reconfigure.server import Server
from rosgraph_msgs.msg import Clock
from sim_clock.cfg import ClockConfig


class SimClock():
    def __init__(self):
        self.start_time = 0.0
        self.cur_time = self.start_time
        self.config = None
        self.server = Server(ClockConfig, self.dr_callback)
        self.pub = rospy.Publisher("clock", Clock, queue_size=3)
        while not rospy.is_shutdown():
            if self.config.play_pause:
                self.update()
                time.sleep(abs(self.config.dt / self.config.real_time_factor))
            else:
                # wait to come out of pause
                time.sleep(0.1)

    def update(self):
        # TODO(lucasw) time values can't be negative, so reverse
        # time isn't going to work well without a large positive offset.
        msg = rospy.Time.from_sec(self.cur_time)
        self.pub.publish(msg)
        # TODO(lucasw) need to track wall time so don't drift behind
        self.cur_time += self.config.dt

    # TODO(lucasw) trigger the update from a message
    # One example of a message would be from a cpu intensive node that has
    # finished processing
    def trigger(self, msg):
        self.update()

    def dr_callback(self, config, level):
        self.config = config
        if not self.config.play_pause and self.config.single_step:
            self.update()
            self.config.single_step = False
        return self.config

if __name__ == '__main__':
    rospy.init_node('sim_clock')
    sim_clock = SimClock()
    # rospy.spin()

