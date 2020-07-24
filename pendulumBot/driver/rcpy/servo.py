#!/usr/bin/env python3
# Run the servers
# import python libraries
import time

import rcpy 
import rcpy.servo as servo

class servo:
  def __init__(self, index):
    self.duty = 0.0

    if index == 1:
      self.m = servo.servo1
    elif index == 2:
      self.m = servo.servo2
    elif index == 3:
      self.m = servo.servo3
    elif index == 4:
      self.m = servo.servo4
    elif index == 5:
      self.m = servo.servo5
    elif index == 6:
      self.m = servo.servo6
    elif index == 7:
      self.m = servo.servo7
    elif index == 8:
      self.m = servo.servo8
    else:
      self.m = None
      return
      
  def set_duty(self, duty):
    self.duty = duty
    self.m.set(duty)
    self.m.enable()
    
  def disable(self):
    self.m.disable()

  def get_duty(self):
    return self.duty
    

if __name__ == "__main__":
  print("Module is not executable")
