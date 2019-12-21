from pendulumBot.bot import robotControl

class RemoteLogCallbacks:
  def __init__(self, logger):
    self._logger = logger

  def register(self, registry):
    registry.add("rlog/active", self.callback_active)

  def callback_active(self, payload):
    if payload[0] == True:
      self._logger.start()
    else:
      self._logger.stop()


class RobotControlCallbacks:
  def __init__(self, control):
    self.control = control

  def register(self, registry):
    registry.add("control/automatic", self.callback_auto   )
    registry.add(  "control/disable", self.callback_disable)
    registry.add(   "control/manual", self.callback_manual )

  def callback_auto(self, payload):
    self.control.set_state(robotControl.RobotControl.STATE_AUTO, payload)
    return True

  def callback_disable(self, payload):
    self.control.set_state(robotControl.RobotControl.STATE_DISABLED, None)
    return True

  def callback_manual(self, payload):
    if len(payload) < 1:
      return False
    if payload[0] == "FW":
      direction = robotControl.RobotControl.MANUAL_DIRECTION_FW
    elif payload[0] == "BW":
      direction = robotControl.RobotControl.MANUAL_DIRECTION_BW
    elif payload[0] == "LT":
      direction = robotControl.RobotControl.MANUAL_DIRECTION_LT
    elif payload[0] == "RT":
      direction = robotControl.RobotControl.MANUAL_DIRECTION_RT
    else:
      return False

    try:
      speed = float(payload[1])
    except:
      speed = 0.5

    duration_ms = 500
    if len(payload) >= 2:
      try:
        duration_ms = int(payload[2]*1000)
      except:
        pass

    self.control.set_state(robotControl.RobotControl.STATE_MANUAL, [direction, speed, duration_ms])
    return True