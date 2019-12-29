#!/usr/bin/env python3
#
# Copyright © 2019 Hoani Bryson
# License: MIT (https://mit-license.org/)
#
# Fake: PID Controller
#
# Pretends to be a PID controller
#

class PidFake:
  def __init__(self, kp=1.0, ki=0.0, kd=0.0, setpoint = 0.0):
    self.kp = kp
    self.ki = ki
    self.kd = kd
    self.setpoint = setpoint
    self.update_called = False
    self.update_args = None
    self.reset_called = False
    self.control_output = 0.0


  def update(self, position, delta_s):
    self.update_called = True
    self.update_args = (position, delta_s)
    return self.control_output

  
  def reset(self):
    self.reset_called = True

  
  def set_setpoint(self, setpoint):
    self.setpoint = setpoint


  def set_kp(self, value):
    self.kp = value  


  def set_ki(self, value):
    self.ki = value  


  def set_kd(self, value):
    self.kd = value  

  ### Fake Methods ###
  def set_control_output(self, output):
    self.control_output = output

        
if __name__ == "__main__":
  print("module not callable")
