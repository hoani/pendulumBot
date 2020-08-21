class MotorPair:
    def __init__(self):
        self.last_call = MotorPair.__init__
        self.speed = 0

    def forward(self, speed):
        self.last_call = MotorPair.forward
        self.speed = speed

    def backward(self, speed):
        self.last_call = MotorPair.backward
        self.speed = speed

    def turn_left(self, speed):
        self.last_call = MotorPair.turn_left
        self.speed = speed

    def turn_right(self, speed):
        self.last_call = MotorPair.turn_right
        self.speed = speed

    def stop(self):
        self.last_call = MotorPair.stop
        self.speed = 0

    def move(self, left, right):
        self.last_call = MotorPair.move
        self.speed = (left, right)
