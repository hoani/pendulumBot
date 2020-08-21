#!/usr/bin/env python3
from dio.driver.interfaces import ServoInterface


class Servo(ServoInterface):
    def __init__(self, index):
        self.duty = 0.0

    def set_duty(self, duty):
        self.duty = duty

    def disable(self):
        self.duty = 0.0

    def get_duty(self):
        return self.duty


if __name__ == "__main__":
    print("Module is not executable")
