# Author: Hoani
#
# Inertial Navigation System

import math

class Direction(complex):
  def __init__(self):
    super().__init__()
    self.real = 1.0
    
  def radian(self):
    return math.asin(self.real/1.0)
  
  def angle(self):
    return 180.0*(self.radian()/math.pi)

class Ins:
  def __init__(self):
    self.direction = Direction()
    
  def get_angle(self):
    return math.sin(self.direction)
    
  def get_direction_vector(self):
    return self.direction