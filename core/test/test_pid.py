#!/usr/bin/env python3
#
# Copyright © 2019 Hoani Bryson
# License: MIT (https://mit-license.org/)
#
# Test: PID Controller
#
# Unit tests for basic PID controller
#

from core import pid


def abs_diff(val1, val2):
    return abs(val1 - val2)


class TestPidBasic:

    def test_initialization(self):
        ctrl = pid.Pid()
        assert(ctrl is not None)

    def test_no_control(self):
        ctrl = pid.Pid(kp=1.0, ki=0.0, kd=0.0, setpoint=0.0)
        u = ctrl.update(position=0.0, delta_s=1.0)
        assert(u == 0.0)

    def test_control_simple(self):
        ctrl = pid.Pid(kp=1.0, ki=0.0, kd=0.0, setpoint=0.0)
        u = ctrl.update(position=-1.0, delta_s=1.0)
        assert(u == 1.0)


class TestPidProportional:

    def setup_method(self):
        self.ctrl = pid.Pid(kp=1.0, ki=0.0, kd=0.0, setpoint=0.0)

    def test_negative(self):
        u = self.ctrl.update(position=1.0, delta_s=1.0)
        assert(u == -1.0)

    def test_fractional(self):
        u = self.ctrl.update(position=-0.5, delta_s=1.0)
        assert(u == 0.5)

    def test_set_kp_high(self):
        self.ctrl.set_kp(25.0)
        u = self.ctrl.update(position=-1.0, delta_s=1.0)
        assert(u == 25.0)

    def test_set_kp_negative(self):
        self.ctrl.set_kp(-1.0)
        u = self.ctrl.update(position=-1.0, delta_s=1.0)
        assert(u == -1.0)

    def test_zero_kp(self):
        self.ctrl.set_kp(0.0)
        u = self.ctrl.update(position=-1.0, delta_s=1.0)
        assert(u == 0.0)

    def test_setpoint_change(self):
        self.ctrl.set_setpoint(10.0)
        u = self.ctrl.update(position=-1.0, delta_s=1.0)
        assert(u == 11.0)


class TestPidIntegral:

    def setup_method(self):
        self.ctrl = pid.Pid(kp=0.0, ki=1.0, kd=0.0, setpoint=0.0)

    def test_simple(self):
        u = self.ctrl.update(position=-1.0, delta_s=1.0)
        assert(u == 1.0)

    def test_setpoint(self):
        self.ctrl.set_setpoint(1.0)
        u = self.ctrl.update(position=0.0, delta_s=1.0)
        assert(u == 1.0)

    def test_fractional_time(self):
        u = self.ctrl.update(position=-1.0, delta_s=0.5)
        assert(u == 0.5)

    def test_accumulates(self):
        self.ctrl.update(position=-1.0, delta_s=0.5)
        self.ctrl.update(position=-1.0, delta_s=0.5)
        self.ctrl.update(position=-1.0, delta_s=0.5)
        u = self.ctrl.update(position=-1.0, delta_s=0.5)
        assert(u == 2.0)

    def test_reset(self):
        self.ctrl.update(position=-1.0, delta_s=1.5)
        self.ctrl.reset()
        u = self.ctrl.update(position=-1.0, delta_s=0.5)
        assert(u == 0.5)

    def test_integration_is_trapezoidal(self):
        self.ctrl.update(position=-1.0, delta_s=1.0)
        u = self.ctrl.update(position=-2.0, delta_s=1.0)
        assert(u == 2.5)

    def test_integral_averages_to_zero(self):
        self.ctrl.update(position=-1.0, delta_s=0.0)
        self.ctrl.update(position=1.0, delta_s=1.0)
        self.ctrl.update(position=-1.0, delta_s=1.0)
        self.ctrl.update(position=1.0, delta_s=1.0)
        u = self.ctrl.update(position=-1.0, delta_s=1.0)
        assert(u == 0.0)

    def test_integral_change_ki(self):
        self.ctrl.update(position=-1.0, delta_s=1.0)
        self.ctrl.set_ki(-10.0)
        u = self.ctrl.update(position=-1.0, delta_s=0.0)
        assert(u == -10.0)


class TestPidDifferential:

    def setup_method(self):
        self.ctrl = pid.Pid(kp=0.0, ki=0.0, kd=1.0, setpoint=0.0)

    def test_first_zero(self):
        u = self.ctrl.update(position=-1.0, delta_s=1.0)
        assert(u == 0.0)

    def test_no_change_zero(self):
        self.ctrl.update(position=-1.0, delta_s=1.0)
        u = self.ctrl.update(position=-1.0, delta_s=1.0)
        assert(u == 0.0)

    def test_simple(self):
        self.ctrl.update(position=0.0, delta_s=1.0)
        u = self.ctrl.update(position=-1.0, delta_s=1.0)
        assert(u == 1.0)

    def test_simple_multicycle(self):
        self.ctrl.update(position=0.0, delta_s=1.0)
        u = self.ctrl.update(position=-1.0, delta_s=1.0)
        u = self.ctrl.update(position=-1.5, delta_s=1.0)
        assert(u == 0.5)

    def test_uses_position_only(self):
        self.ctrl.update(position=0.0, delta_s=1.0)
        self.ctrl.set_setpoint(-1.0)
        u = self.ctrl.update(position=0.0, delta_s=1.0)
        assert(u == 0.0)

    def test_set_kd(self):
        self.ctrl.update(position=0.0, delta_s=1.0)
        self.ctrl.set_kd(15.0)
        u = self.ctrl.update(position=-1.0, delta_s=1.0)
        assert(u == 15.0)

    def test_fractional_time(self):
        self.ctrl.update(position=0.0, delta_s=1.0)
        u = self.ctrl.update(position=-1.0, delta_s=0.125)
        assert(u == 8.0)
