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
from std_msgs.msg import Float32


class SimClock():
    def __init__(self):
        self.start_time = 0.0
        self.cur_time = self.start_time
        self.config = None
        self.server = Server(ClockConfig, self.dr_callback)
        self.pub = rospy.Publisher("clock", Clock, queue_size=100)
        self.sub = rospy.Subscriber("step", Float32, self.step, queue_size=10)
        while not rospy.is_shutdown():
            if self.config.play_pause:
                self.single_dt()
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

    def single_dt(self):
        self.update()
        time.sleep(abs(self.config.dt / self.config.real_time_factor))

    # trigger the update from a message optionally or the
    # One example of a message would be from a cpu intensive node that has
    # finished processing
    def step(self, msg):
        # TODO(lucasw) want to keep a self.target_time around
        # so that sequential calls to step will not round to nearest dt,
        target_time = self.cur_time + msg.data
        # TODO(lucasw) maybe a do-while, so one dt is always going to happen?
        while self.cur_time < target_time and not self.config.play_pause:
            self.single_dt()

    def dr_callback(self, config, level):
        self.config = config
        # TODO(lucasw) is this potentially glitchy at the boundaries
        # with separate thread running in init while loop?
        if not self.config.play_pause and self.config.single_step:
            self.step(Float32(config.step_time))
            self.config.single_step = False
        elif not self.config.play_pause and self.config.single_dt:
            self.single_dt()
            self.config.single_dt = False
        return self.config


if __name__ == '__main__':
    rospy.init_node('sim_clock')
    sim_clock = SimClock()
    # rospy.spin()
