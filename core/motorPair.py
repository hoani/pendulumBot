import numpy as np


class MotorPair:
    def __init__(self, left, right, left_start=0.1, right_start=0.1):
        self.left = left
        self.right = right
        self.l0 = left_start
        self.r0 = right_start
        self.l_scale = 1.0 / (1.0 + left_start)
        self.r_scale = 1.0 / (1.0 + right_start)

    def forward(self, speed):
        self.move(speed, speed)

    def backward(self, speed):
        self.move(-speed, -speed)

    def turn_left(self, speed):
        self.move(-speed, speed)

    def turn_right(self, speed):
        self.move(speed, -speed)

    def stop(self):
        self.left.stop()
        self.right.stop()

    def move(self, left, right):
        left = np.sign(left) * (abs(left) + self.l0) * self.l_scale
        right = np.sign(right) * (abs(right) + self.r0) * self.r_scale
        self.left.run(-left)
        self.right.run(right)

    def get_duty(self):
        return (-self.left.get_duty(), self.right.get_duty())
