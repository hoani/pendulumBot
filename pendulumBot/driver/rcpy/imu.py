
import rcpy.mpu9250 as mpu9250
from pendulumBot.utilities import vect, imuData
import math

class Imu:
  def __init__(self, mapping = vect.Vec3(0, 1, 2), sign = vect.Vec3(1.0, 1.0, 1.0)):
    mpu9250.initialize(enable_magnetometer = True)
    self._mapping = mapping
    self._sign = sign

  def sample(self):
    data = mpu9250.read()
    accelerometer = vect.Vec3(
      self._sign.x * data['accel'][self._mapping.x],
      self._sign.y * data['accel'][self._mapping.y],
      self._sign.z * data['accel'][self._mapping.z]
    )
    gyroscope = vect.Vec3(
      self._sign.x * data['gyro'][self._mapping.x],
      self._sign.y * data['gyro'][self._mapping.y],
      self._sign.z * data['gyro'][self._mapping.z]
    )
    magnetometer = vect.Vec3(
      self._sign.x * data['mag'][self._mapping.x],
      self._sign.y * data['mag'][self._mapping.y],
      self._sign.z * data['mag'][self._mapping.z]
    )
    return imuData.ImuData(accelerometer, gyroscope, magnetometer)

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
