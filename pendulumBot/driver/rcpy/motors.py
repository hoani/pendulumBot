#!/usr/bin/env python3
# Run the motors
# Based on: https://github.com/mcdeoliveira/rcpy/raw/master/examples/rcpy_test_motors.py
# import python libraries
import time

import rcpy 
import rcpy.motor as motor


class dcMotor:
  def __init__(self, index):
    self.duty = 0.0

    if index == 1:
      self.m = motor.motor1
    elif index == 2:
      self.m = motor.motor2
    elif index == 3:
      self.m = motor.motor3
    elif index == 4:
      self.m = motor.motor4
    else:
      self.m = None
      return
        
    print("init motor ", index)
      
  
  # runs motor at a speed between -1.0 to +1.0
  def run(self, speed):
    if self.m == None:
      return
    
    self.duty =  max(-1.0, min(speed, 1.0))
    self.m.set(self.duty)
    
  def stop(self):
    if self.m == None:
      return
    
    self.run(0.0)

  def get_duty(self):
    return self.duty
    

if __name__ == "__main__":
  print("Module is not executable")
