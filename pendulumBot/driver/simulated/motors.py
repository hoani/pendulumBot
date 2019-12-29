#!/usr/bin/env python3
# Run the motors
# Based on: https://github.com/mcdeoliveira/rcpy/raw/master/examples/rcpy_test_motors.py
# import python libraries
import time


class dcMotor:
  def __init__(self, index):
    self.duty = 0.0
      
  def run(self, speed):
    self.duty = speed
    
  def stop(self):
    self.duty = 0.0

  def get_duty(self):
    return self.duty
    

if __name__ == "__main__":
  print("Module is not executable")
