#!/usr/bin/env python3
#
# Copyright © 2019 Hoani Bryson
# License: MIT (https://mit-license.org/)
#
# Fake: Attitude Heading and Reference System
#
# Provides basic AHRS I/O for testing
#

from pendulumBot.utilities import imuData, vect
from pendulumBot.bot.ahrs import AhrsAngles


class AhrsTwoWheeledFake:
  def __init__(self, pitch = 0.0, yaw = 0.0):
    self.angles = AhrsAngles(pitch, yaw)
    self.update_called = False
    self.update_args = None
    self.get_called = False


  def update(self, delta_t, imu_data):
    self.update_called = True
    self.update_args = (delta_t, imu_data)


  def set_angles(self, pitch, yaw):
    self.angles.pitch = pitch
    self.angles.yaw = yaw


  def get(self):
    self.get_called = True
    return self.angles


  def start_calibrate(self):
    pass
  

  def end_calibrate(self):
    return True
  

  def set_still(self):
    pass


  def set_moving(self):
    pass


  def set_smart(self):
    pass







        
if __name__ == "__main__":
  print("module not callable")