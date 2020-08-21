class MotorPair:
    def __init__(self, left, right):
        self.left = left
        self.right = right

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
        self.left.run(-left)
        self.right.run(right)

    def get_duty(self):
        return (-self.left.get_duty(), self.right.get_duty())
