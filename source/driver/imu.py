
import rcpy.mpu9250 as mpu9250

class Vect:
  def __init__(self, x = 0.0, y = 0.0, z = 0.0):
    self.x = x
    self.y = y
    self.z = z
    
class ImuData:
  def __init__(self, accelerometer, gyroscope, magnetometer):
    self.accelerometer = accelerometer
    self.gyroscope = gyroscope
    self.magnetometer = magnetometer

class Imu:
  def __init__(self):
    mpu9250.initialize(enable_magnetometer = True)
    
  def sample(self):
    data = mpu9250.read()
    accelerometer = Vect( 
      data['accel'][0],
      data['accel'][1],
      data['accel'][2]
      )
    gyroscope = Vect( 
      data['gyro'][0],
      data['gyro'][1],
      data['gyro'][2]
      )
    magnetometer = Vect( 
      data['mag'][0],
      data['mag'][1],
      data['mag'][2]
      )
    return ImuData(accelerometer, gyroscope, magnetometer)
    
if __name__ == "__main__":
  import rcpy 
  import time
  import getopt, sys

  rcpy.set_state(rcpy.RUNNING)
  my_imu = Imu()
  print("Press Ctrl-C to exit")
  
  # header
  print("   Accel XYZ (m/s^2) |"
        "    Gyro XYZ (deg/s) |", end='')
  print("  Mag Field XYZ (uT) |", end='')
  print(' Temp (C) | Delta Time (ms)')
  
  current_milli_time = lambda: int(round(time.time() * 1000))
  
  last_ms = current_milli_time()
  
  try:    # keep running
    while True:
      delta_ms = current_milli_time() - last_ms
      last_ms = current_milli_time()
      if rcpy.get_state() == rcpy.RUNNING:
          temp = mpu9250.read_imu_temp()
          data = my_imu.sample()
          print(('\r'
            '{0:6.2f} {1:6.2f} {2:6.2f} |'
            '{3:6.1f} {4:6.1f} {5:6.1f} |'
            '{6:6.1f} {7:6.1f} {8:6.1f} |'
            '   {9:6.1f}| {10:6.1f}'
            ).format(
              data.accelerometer.x,
              data.accelerometer.y,
              data.accelerometer.z,
              data.gyroscope.x,
              data.gyroscope.y,
              data.gyroscope.z,
              data.magnetometer.x,
              data.magnetometer.y,
              data.magnetometer.z,
              temp,
              delta_ms
              ), end='')
      time.sleep(.01)  # sleep some
  except KeyboardInterrupt:
      # Catch Ctrl-C
      pass
  
  finally:
      print("\nBye BeagleBone!")
