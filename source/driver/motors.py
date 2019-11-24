#!/usr/bin/env python3
# Run the motors
# Based on: https://github.com/mcdeoliveira/rcpy/raw/master/examples/rcpy_test_motors.py
# import python libraries
import time

import rcpy 
import rcpy.motor as motor


class dcMotor:
  def __init__(self, index):

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
    
    duty =  max(-1.0, min(speed, 1.0));
    self.m.set(duty)
    
  def stop(self):
    if self.m == None:
      return
    
    self.m.set(0)
    
class dcMotorPair:
  def __init__(self, left, right):
    self.left = dcMotor(left)
    self.right = dcMotor(right)
    
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
    
  

if __name__ == "__main__":
  # defaults
  duty = 0.5
  
  rcpy.set_state(rcpy.RUNNING)
  
  STATE_FW = 0
  STATE_BW = 1
  STATE_LT = 2
  STATE_RT = 3
  
  state = STATE_FW
  
  print("Press Ctrl-C to exit")
      
  try:
    d = 0
    direction = 1
    delta = duty/30
    
    left = 3;
    right = 2;
    
    pair = dcMotorPair(left, right)
    
    
    
    while rcpy.get_state() != rcpy.EXITING: # keep running
    
      if (state == STATE_FW):
        pair.forward(d)
      elif (state == STATE_BW):
        pair.backward(d)
      elif (state == STATE_LT):
        pair.turn_left(d)
      else: # (state == STATE_RT):
        pair.turn_right(d)
     
  
      d = d + direction * delta
      if d > duty:   # end of range?
        direction = -1
      elif d <= 0:
        state += 1
        direction = 1
        if state > STATE_RT:
          state = STATE_FW
          
        if (state == STATE_FW):
          print("Forward")
        elif (state == STATE_BW):
          print("Backward")
        elif (state == STATE_LT):
          print("Left Turn")
        else: # (state == STATE_RT):
          print("Right Turn")
        
              
      time.sleep(.1)  # sleep some
  
  except KeyboardInterrupt:
    # handle what to do when Ctrl-C was pressed
    pass
      
  finally:
    pair.stop()
  
    # say bye
    print("\nBye BeagleBone!")
        
# exiting program will automatically clean up cape
