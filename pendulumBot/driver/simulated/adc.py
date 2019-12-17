import random

def noise(size):
  return size/2.0 - size*random.random()

def battery_voltage():
  return 7.8 + noise(0.1)
