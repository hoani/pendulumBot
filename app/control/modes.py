#!/usr/bin/env python3
#
# Copyright © 2019 Hoani Bryson
# License: MIT (https://mit-license.org/)
#
# Robot Control
#
# Coordinates between various control strategies
#

class RobotControl:
    STATE_DISABLED = 0
    STATE_AUTO = 1
    STATE_MANUAL = 2
    STATE_PENDULUM = 3

    MANUAL_DIRECTION_FW = 0
    MANUAL_DIRECTION_BW = 1
    MANUAL_DIRECTION_LT = 2
    MANUAL_DIRECTION_RT = 3

    AUTO_STATE_FW = 0
    AUTO_STATE_BW = 1
    AUTO_STATE_LT = 2
    AUTO_STATE_RT = 3

    AUTO_DUTY = 0.5
    AUTO_DELTA = 0.5/32

    def __init__(self, wheels):
        self.state = RobotControl.STATE_DISABLED
        self.auto_state = RobotControl.AUTO_STATE_FW
        self.auto_accel = 1.0
        self.auto_speed_multiplier = 0.5
        self.auto_speed = 0.0
        self.manual_direction = RobotControl.MANUAL_DIRECTION_FW
        self.manual_speed = 0.0
        self.manual_period_ms = 500
        self.controller_map = {}

        self.wheels = wheels

    def add_controller(self, state, controller):
        self.controller_map[state] = controller

    def set_state(self, state):
        if state == RobotControl.STATE_DISABLED:
            print("Robo: DISABLED")
            self.wheels.stop()

        elif state == RobotControl.STATE_AUTO:
            print("Robo: AUTO Mode")
            self.auto_state = RobotControl.AUTO_STATE_FW
            self.auto_accel = 1
            self.auto_speed = 0

        elif state == RobotControl.STATE_MANUAL:
            print("Robo: MANUAL Mode")

        elif state == RobotControl.STATE_PENDULUM:
            print("Robo: PENDULUM Mode")

        self.state = state

    def set_manual_direction(self, direction):
        self.manual_direction = direction

    def set_manual_speed(self, speed):
        self.manual_speed = speed

    def set_manual_duration_ms(self, duration_ms):
        self.manual_period_ms = duration_ms

    def set_automatic_speed(self, speed):
        self.auto_speed_multiplier = speed

    def update(self, delta_ms):
        if self.state in self.controller_map.keys():
            controller = self.controller_map[self.state]
            controller.update(self.wheels, delta_ms)
        else:

            if self.state == RobotControl.STATE_DISABLED:
                pass
            elif self.state == RobotControl.STATE_AUTO:
                self.update_auto(delta_ms)
            elif self.state == RobotControl.STATE_MANUAL:
                self.update_manual(delta_ms)

    def update_auto(self, delta_ms):
        speed = self.auto_speed * self.auto_speed_multiplier
        if (self.auto_state == RobotControl.AUTO_STATE_FW):
            self.wheels.forward(speed)

        elif (self.auto_state == RobotControl.AUTO_STATE_BW):
            self.wheels.backward(speed)

        elif (self.auto_state == RobotControl.AUTO_STATE_LT):
            self.wheels.turn_left(speed)

        else:  # (self.auto_state == RobotControl.AUTO_STATE_RT):
            self.wheels.turn_right(speed)

        self.auto_speed = self.auto_speed + \
            self.auto_accel * RobotControl.AUTO_DELTA

        if self.auto_speed > RobotControl.AUTO_DUTY:   # end of range?
            self.auto_accel = -1

        elif self.auto_speed <= 0:
            self.auto_state += 1
            self.auto_accel = 1

            if self.auto_state > RobotControl.AUTO_STATE_RT:
                self.auto_state = RobotControl.AUTO_STATE_FW

            if (self.auto_state == RobotControl.AUTO_STATE_FW):
                print("Auto: Forward")

            elif (self.auto_state == RobotControl.AUTO_STATE_BW):
                print("Auto: Backward")

            elif (self.auto_state == RobotControl.AUTO_STATE_LT):
                print("Auto: Left Turn")

            else:  # (self.auto_state == AUTO_STATE_RT):
                print("Auto: Right Turn")

    def update_manual(self, delta_ms):
        print(self.manual_period_ms)
        self.manual_period_ms -= delta_ms

        if self.manual_period_ms <= 0:
            self.manual_period_ms = 0
            self.set_state(RobotControl.STATE_DISABLED)
            return

        if (self.manual_direction == RobotControl.MANUAL_DIRECTION_FW):
            self.wheels.forward(self.manual_speed)

        elif (self.manual_direction == RobotControl.MANUAL_DIRECTION_BW):
            self.wheels.backward(self.manual_speed)

        elif (self.manual_direction == RobotControl.MANUAL_DIRECTION_LT):
            self.wheels.turn_left(self.manual_speed)

        else:  # (self.manual_direction == RobotControl.MANUAL_DIRECTION_RT):
            self.wheels.turn_right(self.manual_speed)

    def get_wheels_duty(self):
        return self.wheels.get_duty()


if __name__ == "__main__":
    print("module not callable")
