#!/usr/bin/env python3
#
# Copyright © 2019 Hoani Bryson
# License: MIT (https://mit-license.org/)
#
# Pendulum
#
# Coordinates PID control to hold a setpoint in pendulum control
#

class PendulumPid:
    def __init__(self, pid=None, ahrs=None, limit=5.0):
        self.limit = limit
        self.pid = pid
        self.ahrs = ahrs
        self.enable = False
        self.suspend = True

    def update(self, wheels, delta_ms):
        if not self.enable:
            return

        angles = self.ahrs.get()

        self._update_suspend(angles.pitch)

        if self.suspend:
            wheels.stop()
            return

        control = self.pid.update(
            position=angles.pitch,
            delta_s=delta_ms * 0.001
        )

        control = min(max(-1.0, control), 1.0)

        if control != 0.0:
            wheels.move(control, control)
        else:
            wheels.stop()

    def set_kp(self, value):
        return self._set_gain(self.pid.set_kp, value)

    def set_ki(self, value):
        return self._set_gain(self.pid.set_ki, value)

    def set_kd(self, value):
        return self._set_gain(self.pid.set_kd, value)

    def _set_gain(self, callback, value):
        try:
            value = float(value)
            callback(value)
            return True
        except Exception:
            self._print_set_failure("gain", value)
            return False

    def set_setpoint(self, setpoint):
        try:
            setpoint = float(setpoint)
            self.pid.set_setpoint(setpoint)
            return True
        except Exception:
            self._print_set_failure("setpoint", setpoint)
            return False

    def set_limit(self, limit):
        try:
            self.limit = abs(float(limit))
            return True
        except Exception:
            self._print_set_failure("limit", limit)
            return False

    def set_enable(self, enable):
        if isinstance(enable, bool):
            self.enable = enable
            self.suspend = False
            if self.enable:
                self.pid.reset()
            return True
        else:
            self._print_set_failure("enable", enable)
            return False

    def _update_suspend(self, pitch):
        if not self.suspend:
            if abs(pitch) >= self.limit:
                self.suspend = True

        else:
            if abs(pitch) <= 0.25 * self.limit:
                self.suspend = False
                self.pid.reset()

    def _print_set_failure(self, settable, value):
        print("")
        print("control/pendulum.py:")
        print("failed to set invalid {} [{}]".format(settable, value))
        print("")


if __name__ == "__main__":
    print("module not callable")
