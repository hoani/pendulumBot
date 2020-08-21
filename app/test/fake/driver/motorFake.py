#!/usr/bin/env python3


class dcMotor:
    def __init__(self, index):
        self.index = index
        self.speed = 0

    # runs motor at a speed between -1.0 to +1.0
    def run(self, speed):
        self.speed = speed

    def stop(self):
        self.speed = 0
