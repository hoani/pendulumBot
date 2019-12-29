from pendulumBot.bot import robotControl

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
    self.control.set_state(robotControl.RobotControl.STATE_MANUAL)
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
      return True
    except:
      return False



if __name__ == "__main__":
  print("module not callable")
  
