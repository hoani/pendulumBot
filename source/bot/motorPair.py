class dcMotorPair:
  def __init__(self, left, right):
    self.left = left
    self.right = right
    
  def forward(self, speed):
    self.left.run(-speed)
    self.right.run(speed)
  
  def backward(self, speed):
    self.left.run(speed)
    self.right.run(-speed)
    
  def turn_left(self, speed):
    self.left.run(speed)
    self.right.run(speed)
    
  def turn_right(self, speed):
    self.left.run(-speed)
    self.right.run(-speed)
    
  def stop(self):
    self.left.stop()
    self.right.stop()