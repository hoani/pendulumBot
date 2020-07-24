#!/usr/bin/env python3
# Run the servers
# import python libraries
import time

import rcpy 
import rcpy.servo as rcpyservo

class servo:
  def __init__(self, index):
    self.duty = 0.0
    self.clock = None
    rcpyservo.enable()

    if index == 1:
      self.m = rcpyservo.servo1
    elif index == 2:
      self.m = rcpyservo.servo2
    elif index == 3:
      self.m = rcpyservo.servo3
    elif index == 4:
      self.m = rcpyservo.servo4
    elif index == 5:
      self.m = rcpyservo.servo5
    elif index == 6:
      self.m = rcpyservo.servo6
    elif index == 7:
      self.m = rcpyservo.servo7
    elif index == 8:
      self.m = rcpyservo.servo8
    else:
      self.m = None
      return
      
  def set_duty(self, duty):
    if self.clock is None:
      self.clock = self.m.start(0.1)
    self.duty = duty
    self.m.set(duty)
    
  def disable(self):
    self.clock.stop()
    self.clock = None

  def get_duty(self):
    return self.duty
    

if __name__ == "__main__":
  print("Module is not executable")
