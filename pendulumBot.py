import time, sys, os
from pendulumBot.bot.robotControl import *
from pendulumBot.bot import runner


import json

def get_settings(settings_file_path):
  settings_file_path = 'config/settings.json'
  with open(settings_file_path, "r") as settings_file:
    settings = json.load(settings_file)
  return settings



if __name__ == "__main__":
  settings = get_settings('config/settings.json')
  _runner = runner.RobotRunner(settings)
  delta_ms = 20
  _runner.run(delta_ms)
  print("\nBye BeagleBone!")


