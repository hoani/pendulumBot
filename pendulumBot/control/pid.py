#!/usr/bin/env python3
#
# Copyright © 2019 Hoani Bryson
# License: MIT (https://mit-license.org/)
#
# PID Controller
#
# Basic PID controller
# Uses derivative of the input value rather than error term to avoid control 
# spikes when the setpoint changes
#

class Pid:

  def __init__(self, kp=1.0, ki=0.0, kd=0.0, setpoint = 0.0):
    self.kp = kp
    self.ki = ki
    self.kd = kd
    self.setpoint = setpoint
    self.error_last = None
    self.position_last = None
    self.integrated_error = 0.0
    self.control_output = 0.0


  def update(self, position, delta_s):
    error = self.setpoint - position
    
    pout = self._calculate_proportional(position, delta_s, error)
    iout = self._calculate_integral(position, delta_s, error)
    dout = self._calculate_derivative(position, delta_s, error)
    
    self.error_last = error

    self.control_output = pout + iout + dout
    return self.control_output

  
  def reset(self):
    self.integrated_error = 0.0
    self.error_last = None

  
  def set_setpoint(self, setpoint):
    self.setpoint = setpoint


  def set_kp(self, value):
    self.kp = value  


  def set_ki(self, value):
    self.ki = value  


  def set_kd(self, value):
    self.kd = value  


  def _calculate_proportional(self, position, delta_s, error):
    return self.kp * error


  def _calculate_integral(self, position, delta_s, error):
    if self.ki != 0.0:
      if self.error_last == None:
        self.error_last = error

      self.integrated_error += (self.error_last + error)/2.0 * delta_s
      return self.ki * self.integrated_error
    
    return 0.0


  def _calculate_derivative(self, position, delta_s, error):
    if self.kd != 0.0:
      if self.position_last == None:
        self.position_last = position

      if delta_s > 0.0:
        dpos = self.position_last - position
        return self.kd * dpos / delta_s
    
    return 0.0


    
        
if __name__ == "__main__":
  print("module not callable")
