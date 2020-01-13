#!/usr/bin/env python
import numpy

class DiscreteIntegrator(object):

    def __init__(self, start_time, initial_value):

        self.cur_time = start_time
        self.init_val = initial_value
        self.last_time = start_time
        self.cur_val = initial_value
        self.last_meas = initial_value

    def euler(self, meas, cur_time, reset=False):

        if reset:
            self.cur_val = 0.0
            self.last_time = cur_time

        self.cur_time = cur_time
        self.cur_val = self.init_val + meas * (self.cur_time - self.last_time)
        self.init_val = self.cur_val
        self.last_time = self.cur_time

    def trapezoidal(self, meas, cur_time, reset=False):

        if reset:
            self.cur_val = 0.0
            self.last_time = cur_time

        self.cur_time = cur_time
        self.cur_val = (self.cur_time - self.last_time)/2 * (meas - self.last_meas)
        self.last_time = cur_time
        self.last_meas = meas



# test functionality
t0 = 0
init_val = 0
Integrator = DiscreteIntegrator(t0, init_val)
Integrator.euler(2, 0.1)
print(Integrator.cur_val)
