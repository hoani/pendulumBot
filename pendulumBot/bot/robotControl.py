#!/usr/bin/env python3

class RobotControl:
  STATE_DISABLED = 0
  STATE_AUTO = 1
  STATE_MANUAL = 2
  
  MANUAL_DIRECTION_FW = 0
  MANUAL_DIRECTION_BW = 1
  MANUAL_DIRECTION_LT = 2
  MANUAL_DIRECTION_RT = 3
  
  AUTO_STATE_FW = 0
  AUTO_STATE_BW = 1
  AUTO_STATE_LT = 2
  AUTO_STATE_RT = 3
  
  AUTO_DUTY = 0.5
  AUTO_DELTA = 0.5/32
    
  
  def __init__(self, wheels):
    self.state = RobotControl.STATE_DISABLED
    self.auto_state = RobotControl.AUTO_STATE_FW
    self.auto_accel = 1
    self.auto_speed = 0
    self.manual_direction = RobotControl.MANUAL_DIRECTION_FW
    self.manual_speed = 0.0
    self.manual_period_ms = 500
    
    self.wheels = wheels
    
  def set_state(self, state, payload):
    if state == RobotControl.STATE_DISABLED:
      print("Robo: DISABLED")
      self.wheels.stop()
      
    if state == RobotControl.STATE_AUTO:
      print("Robo: AUTO Mode")
      self.auto_state = RobotControl.AUTO_STATE_FW
      self.auto_accel = 1
      self.auto_speed = 0
    
    elif state == RobotControl.STATE_MANUAL:
      print("Robo: MANUAL Mode", payload)
      self.manual_period_ms = 500
      if len(payload) > 0:
        self.manual_direction = payload[0]
        if len(payload) > 1:
          self.manual_speed = payload[1]
          if len(payload) > 2:
            self.manual_period_ms = payload[2]
      print(self.manual_period_ms, payload[2])
      self.update_manual(0)
      

    self.state = state
    
    
  def update(self, delta_ms):
    if self.state == RobotControl.STATE_DISABLED:
      pass
    elif self.state == RobotControl.STATE_AUTO:
      self.update_auto(delta_ms)
    elif self.state == RobotControl.STATE_MANUAL:
      self.update_manual(delta_ms)
      
      
  def update_auto(self, delta_ms):
    if (self.auto_state == RobotControl.AUTO_STATE_FW):
      self.wheels.forward(self.auto_speed)
      
    elif (self.auto_state == RobotControl.AUTO_STATE_BW):
      self.wheels.backward(self.auto_speed)
      
    elif (self.auto_state == RobotControl.AUTO_STATE_LT):
      self.wheels.turn_left(self.auto_speed)
      
    else: # (self.auto_state == RobotControl.AUTO_STATE_RT):
      self.wheels.turn_right(self.auto_speed)
   
    self.auto_speed = self.auto_speed + self.auto_accel * RobotControl.AUTO_DELTA
    
    if self.auto_speed > RobotControl.AUTO_DUTY:   # end of range?
      self.auto_accel = -1
      
    elif self.auto_speed <= 0:
      self.auto_state += 1
      self.auto_accel = 1
      
      if self.auto_state > RobotControl.AUTO_STATE_RT:
        self.auto_state = RobotControl.AUTO_STATE_FW
        
      if (self.auto_state == RobotControl.AUTO_STATE_FW):
        print("Auto: Forward")
        
      elif (self.auto_state == RobotControl.AUTO_STATE_BW):
        print("Auto: Backward")
        
      elif (self.auto_state == RobotControl.AUTO_STATE_LT):
        print("Auto: Left Turn")
        
      else: # (self.auto_state == AUTO_STATE_RT):
        print("Auto: Right Turn")
        
  def update_manual(self, delta_ms):
    self.manual_period_ms -= delta_ms
    
    if self.manual_period_ms <= 0:
      self.manual_period_ms = 0
      self.set_state(RobotControl.STATE_DISABLED, [])
      return
    
    if (self.manual_direction == RobotControl.MANUAL_DIRECTION_FW):
      self.wheels.forward(self.manual_speed)
      
    elif (self.manual_direction == RobotControl.MANUAL_DIRECTION_BW):
      self.wheels.backward(self.manual_speed)
      
    elif (self.manual_direction == RobotControl.MANUAL_DIRECTION_LT):
      self.wheels.turn_left(self.manual_speed)
      
    else: # (self.manual_direction == RobotControl.MANUAL_DIRECTION_RT):
      self.wheels.turn_right(self.manual_speed)
        
        
if __name__ == "__main__":
  print("module not callable")