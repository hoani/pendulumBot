class MotorPair:
  def __init__(self):
    self.last_call = MotorPair.__init__
    self.speed = 0
    
  def forward(self, speed):
    self.mode = MotorPair.forward
    self.speed = speed
  
  def backward(self, speed):
    self.mode = MotorPair.backward
    self.speed = speed
    
  def turn_left(self, speed):
    self.mode = MotorPair.turn_left
    self.speed = speed
    
  def turn_right(self, speed):
    self.mode = MotorPair.turn_right
    self.speed = speed
    
  def stop(self):
    self.mode = MotorPair.stop
    self.speed = 0