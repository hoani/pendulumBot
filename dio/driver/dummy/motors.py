#!/usr/bin/env python3

from dio.driver.interfaces import MotorInterface


class DcMotor(MotorInterface):
    def __init__(self):
        self.duty = 0.0

    def run(self, speed):
        self.duty = speed

    def stop(self):
        self.duty = 0.0

    def get_duty(self):
        return self.duty


if __name__ == "__main__":
    print("Module is not executable")
