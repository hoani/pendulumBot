#!/usr/bin/env python3
#
# Copyright © 2019 Hoani Bryson
# License: MIT (https://mit-license.org/)
#
# Command Callbacks
#
# Classes to handle commands and thier associated callbacks.
# Currently all command callbacks are in one place, this may
# change as the code base evolves.
#

from pendulumBot.bot import robotControl


class PendulumCallbacks:
  def __init__(self, control, pendulum):
    self._control = control
    self._pendulum = pendulum

  def register(self, registry):
    registry.add("control/pendulum/gains/kp", self.callback_gain_kp)
    registry.add("control/pendulum/gains/ki", self.callback_gain_ki)
    registry.add("control/pendulum/gains/kd", self.callback_gain_kd)
    registry.add("control/pendulum/setpoint", self.callback_setpoint)
    registry.add("control/pendulum/limit", self.callback_limit)
    registry.add("control/pendulum/enable", self.callback_enable)

  def callback_gain_kp(self, payload):
    return self._pendulum.set_kp(payload)

  def callback_gain_ki(self, payload):
    return self._pendulum.set_ki(payload)

  def callback_gain_kd(self, payload):
    return self._pendulum.set_kd(payload)

  def callback_setpoint(self, payload):
    return self._pendulum.set_setpoint(payload)

  def callback_limit(self, payload):
    return self._pendulum.set_limit(payload)

  def callback_enable(self, payload):
    try:
      enable = bool(payload)
      if enable:
        self._control.set_state(robotControl.RobotControl.STATE_PENDULUM)
      else:
        self._control.set_state(robotControl.RobotControl.STATE_DISABLED)
      return self._pendulum.set_enable(enable)
    except:
      return False


class AhrsCallbacks:
  def __init__(self, ahrs):
    self._ahrs = ahrs

  def register(self, registry):
    registry.add("ahrs/mode/cal",     self.callback_calibrate)
    registry.add("ahrs/mode/still",   self.callback_still)
    registry.add("ahrs/mode/dynamic", self.callback_dynamic)
    registry.add("ahrs/mode/smart",   self.callback_smart)

  def callback_calibrate(self, payload):    
    if payload == True:
      self._ahrs.set_calibrate()
      return True
    else:
      return False
      
  def callback_still(self, payload):    
    if payload == True:
      self._ahrs.set_still()
      return True
    else:
      return False

  def callback_dynamic(self, payload):    
    if payload == True:
      self._ahrs.set_dynamic()
      return True
    else:
      return False

  def callback_smart(self, payload):    
    if payload == True:
      self._ahrs.set_smart()
      return True
    else:
      return False


class RemoteLogCallbacks:
  def __init__(self, logger):
    self._logger = logger

  def register(self, registry):
    registry.add("rlog/active", self.callback_active)

  def callback_active(self, payload):
    if payload == True:
      self._logger.start()
    else:
      self._logger.stop()


class RobotControlCallbacks:
  def __init__(self, control):
    self.control = control

  def register(self, registry):
    registry.add( "control/automatic/speed",  self.callback_automatic_speed)
    registry.add( "control/automatic/enable", self.callback_automatic_enable)
    registry.add( "control/disable",          self.callback_disable)
    registry.add( "control/manual/direction", self.callback_manual_direction )
    registry.add( "control/manual/speed",     self.callback_manual_speed )
    registry.add( "control/manual/duration",  self.callback_manual_duration )

  def callback_automatic_speed(self, payload):
    try:
      speed = float(payload)
      
      self.control.set_automatic_speed(speed)
      return True
    except:
      return False

  def callback_automatic_enable(self, payload):
    try:
      automatic = bool(payload)
      if automatic:
        self.control.set_state(robotControl.RobotControl.STATE_AUTO)
        return True
      else:
        return False
    except:
      return False

  def callback_disable(self, payload):
    self.control.set_state(robotControl.RobotControl.STATE_DISABLED)
    return True

  def callback_manual_direction(self, payload):
    if len(payload) < 1:
      return False
    if payload == "FW":
      direction = robotControl.RobotControl.MANUAL_DIRECTION_FW
    elif payload == "BW":
      direction = robotControl.RobotControl.MANUAL_DIRECTION_BW
    elif payload == "LT":
      direction = robotControl.RobotControl.MANUAL_DIRECTION_LT
    elif payload == "RT":
      direction = robotControl.RobotControl.MANUAL_DIRECTION_RT
    else:
      return False

    self.control.set_manual_direction(direction)
    return True

  def callback_manual_speed(self, payload):
    try:
      speed = float(payload)
      self.control.set_manual_speed(speed)
      return True
    except:
      return False

  def callback_manual_duration(self, payload):
    try:
      duration_ms = int(payload * 1000)
      self.control.set_manual_duration_ms(duration_ms)
      self.control.set_state(robotControl.RobotControl.STATE_MANUAL)
      return True
    except:
      return False



if __name__ == "__main__":
  print("module not callable")
  