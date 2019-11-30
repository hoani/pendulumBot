from pendulumBot.utilities import vect, imuData
import random

def noise(size):
  return size/2 - size*random.random()

class Imu:
  def __init__(self):
    pass
    
  def sample(self):
    accelerometer = vect.Vec3( 
      0.0 + noise(0.8),
      0.0 + noise(0.6),
      9.8 + noise(0.8)
      )
    gyroscope = vect.Vec3( 
      0.0 + noise(1.0),
      0.0 + noise(1.0),
      0.0 + noise(1.2)
      )
    magnetometer = vect.Vec3( 
      10 + noise(5.0),
      20 + noise(4.0),
      -20 + noise(5.0)
      )
    return imuData.ImuData(accelerometer, gyroscope, magnetometer)
    
if __name__ == "__main__":
  print("This module is not executable")
