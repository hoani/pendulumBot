# Author: Hoani
#
# Attitude Heading and Reference System - two wheeled
#
# Takes accelerometer, gyroscopic and magnetometer data and determines
# Robot angle with respect to the ground and the robot's pitch angle
from pendulumBot.utilities import imuData, vect
import numpy as np
import copy

def unit_vector(vector):
  return vector / np.linalg.norm(vector)


def angle_2d_deg(primary, secondary):
  p_u = unit_vector(primary)
  s_u = unit_vector(secondary)
  abs_radians = np.arccos(np.clip(np.dot(p_u, s_u), -1.0, 1.0))
  polarity = np.sign(np.cross(p_u, s_u))

  if polarity != 0: # happens when cross product = 0
    radians = abs_radians * polarity
  else:
    radians = abs_radians
  return radians * AhrsConstants.RAD_TO_DEG


class AhrsAngles:
  def __init__(self, pitch, yaw):
    self.pitch = pitch
    self.yaw = yaw


class AhrsConstants:
  DEFAULT_GRAVITY = 9.81
  GRAVITY_BOUNDS = (9.41, 10.21)
  CALIBRATION_GRAVITY_MAX = 10.81
  CALIBRATION_GYRO_MAX = 10.0
  DEG_WRAP = 180.0
  DEG_PER_REV = 360.0
  DEG_TO_RAD = 2.0 * np.pi / 360.0
  RAD_TO_DEG = 360.0 / (2.0 * np.pi)
  MAG_AVERAGE = 10
  ACCEL_AVERAGE = 10


class CalibrationData:
  def __init__(self):
    self.count = 0
    self.data = None
    self.magnetometer_zero = None
    self.accelerometer_grav = AhrsConstants.DEFAULT_GRAVITY
    self.gyro_drift = None
    self.calibrated = False
    self.max_gyro = 0.0
    self.max_accel = 0.0


class AhrsTwoWheeled:
  # Modes are used to supply additional information for the robot to infer parameters
  #   MODE_DYNAMIC:
  #     Used in situations when the robot is moving, the algorithms rely only on the gyros
  #     This is because:
  #       * the accelerometers cannot be relied on as providing a reliable gravity vector 
  #         without using quaternions (potential improvement here)
  #       * the magnetometer has low bandwidth and is susceptable to noise 
  #         (esp with motors running) 
  #   MODE_SMART:
  #     Same as mode dynamic for yaw
  #     Uses a simple sensor fusion algorithm for pitch:
  #       If the absolute acceleration value is outside of gravity 
  #       (given a small margin of 0.2(m/s2)), yaw is calculated from gyro only,
  #       Otherwise, the yaw is calculated from a combination of both in order to limit
  #       the drift caused by yaw
  #   MODE_STILL:
  #     Corrects for integration error in pitch calculation by using accelerometers
  #     Corrects for integration error in yaw calculation by using magnetometers
  #   MODE_CALIBRATE:
  #     Assumes the robot is completely still and will not move until after calibration
  #     is complete
  #     Switches to MODE_STILL with no updates to calibrated values if high accelerations or 
  #     angular rates are detected
  #     When calibrate is complete, the yaw angle is set to zero.
  MODE_DYNAMIC = 0
  MODE_SMART = 1
  MODE_STILL = 2 
  MODE_CALIBRATE = 3

  def __init__(self, transform=[[1,0,0],[0,1,0],[0,0,1]]):
    self._offsets = imuData.ImuData(
      vect.Vec3(),
      vect.Vec3(),
      vect.Vec3()
    )
    self._last_data = None
    self._transform = transform
    self.gravity = AhrsConstants.DEFAULT_GRAVITY
    self.yaw = 0.0
    self.pitch = 0.0
    self._mode = AhrsTwoWheeled.MODE_DYNAMIC
    self._mag_zero_matrix = None
    self._update_map = {
      AhrsTwoWheeled.MODE_DYNAMIC:    self._update_dynamic,
      AhrsTwoWheeled.MODE_SMART:      self._update_dynamic,
      AhrsTwoWheeled.MODE_STILL:      self._update_still,
      AhrsTwoWheeled.MODE_CALIBRATE:  self._update_calibrate
    }
    self._cal = CalibrationData()
    self._mag_data = vect.Vec3(
      np.zeros(AhrsConstants.MAG_AVERAGE),
      np.zeros(AhrsConstants.MAG_AVERAGE), 
      np.zeros(AhrsConstants.MAG_AVERAGE)
    )
    self._accel_data = vect.Vec3(
      np.zeros(AhrsConstants.ACCEL_AVERAGE),
      np.zeros(AhrsConstants.ACCEL_AVERAGE), 
      np.zeros(AhrsConstants.ACCEL_AVERAGE)
    )
    self._still_count = 0


  def update(self, delta_t, imu_data):
    self._store_imu_data(imu_data)
    self._update_map[self._mode](delta_t, imu_data)


  def _update_dynamic(self, delta_t, imu_data):
    transform = self._gyro_matrix()
    gyro_matrix = imu_data.gyroscope.array()

    rates = transform.dot(gyro_matrix)
    rate_pitch = rates.item(1)
    rate_yaw = rates.item(2)
    
    gyro_yaw = self._integrate_angle(delta_t, self.yaw, rate_yaw)
    self.yaw = gyro_yaw

    gyro_pitch = self._integrate_angle(delta_t, self.pitch, rate_pitch)
    self.pitch = gyro_pitch


  def _update_smart(self, delta_t, imu_data):
    transform = self._gyro_matrix()
    gyro_matrix = imu_data.gyroscope.array()

    rates = transform.dot(gyro_matrix)
    rate_pitch = rates.item(1)
    rate_yaw = rates.item(2)

    accel_pitch = None
    mag_yaw = None

    if self._cal.calibrated:  
      if np.linalg.norm(imu_data.gyroscope.array()) >= AhrsConstants.CALIBRATION_GYRO_MAX:
        self._still_count = 0
      else:
        self._still_count += 1

      if self._still_count >= AhrsConstants.ACCEL_AVERAGE:
        accel_pitch = self._calculate_pitch_from_accelerometer(
          self._sum_accelerometer_data()
        )

      if self._still_count >= AhrsConstants.MAG_AVERAGE:
        s = slice(0, 2)  # Only care about x and y coordinates here
        transform = self._gyro_matrix()
        mag_avg = np.array([
          np.average(self._mag_data.x),
          np.average(self._mag_data.y),
          np.average(self._mag_data.z)
        ])
        mag_global = transform.dot(mag_avg)
        mag_yaw = angle_2d_deg(
          mag_global[s],
          self._cal.magnetometer_zero.array()[s]
        )

    if mag_yaw != None:
      self.yaw = mag_yaw
    gyro_yaw = self._integrate_angle(delta_t, self.yaw, rate_yaw)
    self.yaw = gyro_yaw

    if accel_pitch != None:
      self.pitch = accel_pitch
    gyro_pitch = self._integrate_angle(delta_t, self.pitch, rate_pitch)
    self.pitch = gyro_pitch


  def _update_still(self, delta_t, imu_data):
    if np.linalg.norm(imu_data.gyroscope.array()) >= AhrsConstants.CALIBRATION_GYRO_MAX:
      self._still_count = 0
    else:
      self._still_count += 1
    
    if self._cal.calibrated:  

      if self._still_count >= AhrsConstants.ACCEL_AVERAGE:
        self.pitch = self._calculate_pitch_from_accelerometer(
          self._sum_accelerometer_data()
        )

      if self._still_count >= AhrsConstants.MAG_AVERAGE:
        s = slice(0, 2)  # Only care about x and y coordinates here
        transform = self._gyro_matrix()
        mag_avg = np.array([
          np.average(self._mag_data.x),
          np.average(self._mag_data.y),
          np.average(self._mag_data.z)
        ])
        mag_global = transform.dot(mag_avg)
        mag_yaw = angle_2d_deg(
          mag_global[s],
          self._cal.magnetometer_zero.array()[s]
        )
        self.yaw = mag_yaw

  

  def _update_calibrate(self, delta_t, imu_data):
    if self._cal.data == None:
      self._cal.data = copy.deepcopy(imu_data)
    else:
      self._cal.data.accelerometer.x += imu_data.accelerometer.x
      self._cal.data.accelerometer.y += imu_data.accelerometer.y
      self._cal.data.accelerometer.z += imu_data.accelerometer.z
      self._cal.data.gyroscope.x     += imu_data.gyroscope.x
      self._cal.data.gyroscope.y     += imu_data.gyroscope.y
      self._cal.data.gyroscope.z     += imu_data.gyroscope.z
      self._cal.data.magnetometer.x  += imu_data.magnetometer.x
      self._cal.data.magnetometer.y  += imu_data.magnetometer.y
      self._cal.data.magnetometer.z  += imu_data.magnetometer.z

    self._cal.max_gyro = max(
      self._cal.max_gyro, 
      np.linalg.norm(imu_data.gyroscope.array())
    )

    self._cal.max_accel = max(
      self._cal.max_accel, 
      np.linalg.norm(imu_data.accelerometer.array())
    )

    self._cal.count += 1
    pass


  def get(self):
    return AhrsAngles(self.pitch, self.yaw)


  def start_calibrate(self):
    self._mode = AhrsTwoWheeled.MODE_CALIBRATE
    self._cal = CalibrationData()
  

  def end_calibrate(self):
    # Check for bad calibration data
    accel_norm = np.linalg.norm(
      self._cal.data.accelerometer.array()
    )

    accel_avg = accel_norm / self._cal.count

    bounds = AhrsConstants.GRAVITY_BOUNDS

    if accel_avg != np.clip(accel_avg, bounds[0], bounds[1]):
      return False
    
    if self._cal.max_accel > AhrsConstants.CALIBRATION_GRAVITY_MAX:
      return False
    
    if self._cal.max_gyro > AhrsConstants.CALIBRATION_GYRO_MAX:
      return False

    # We assume at calibration yaw = 0.0 
    self.yaw = 0.0

    # The pitch angle is derived from the accelerometer
    self.pitch = self._calculate_pitch_from_accelerometer(
      self._cal.data.accelerometer
    )

    # Set calibrated gravity value
    self._cal.accelerometer_grav = accel_avg

    # Map calibrated magnetometer to global coordinates
    transform = self._gyro_matrix()
    mag_sampled = self._cal.data.magnetometer.array()
    
    mag_zero = transform.dot(mag_sampled)

    # Store the calibrated and mapped magnetometer values
    self._cal.magnetometer_zero = vect.Vec3(
      mag_zero[0], 
      mag_zero[1], 
      mag_zero[2]
    )

    self._cal.gyro_drift = vect.Vec3(
      self._cal.data.gyroscope.x / self._cal.count,
      self._cal.data.gyroscope.y / self._cal.count,
      self._cal.data.gyroscope.z / self._cal.count
    )

    # Change state
    self._mode = AhrsTwoWheeled.MODE_STILL
    self._cal.calibrated = True

    return True
  

  def set_still(self):
    self._mag_count = 0
    self._mode = AhrsTwoWheeled.MODE_STILL


  def set_moving(self):
    self._mode = AhrsTwoWheeled.MODE_DYNAMIC


  def set_smart(self):
    self._mode = AhrsTwoWheeled.MODE_SMART


  def _gyro_matrix(self):
    k = AhrsConstants.DEG_TO_RAD
    sin_pitch = np.sin(k * self.pitch)
    cos_pitch = np.cos(k * self.pitch)
    return np.array([
      [  cos_pitch, 0.0, sin_pitch ],
      [        0.0, 1.0,       0.0 ],
      [ -sin_pitch, 0.0, cos_pitch ]
    ])

  def _integrate_angle(self, delta_t, current_angle, rate):
      angle = current_angle + rate * delta_t
      while(angle > AhrsConstants.DEG_WRAP):
        angle -= AhrsConstants.DEG_PER_REV
      while(angle < -AhrsConstants.DEG_WRAP):
        angle += AhrsConstants.DEG_PER_REV
      return angle

  def _sum_accelerometer_data(self):
    return vect.Vec3(
      np.sum(self._accel_data.x),
      np.sum(self._accel_data.y),
      np.sum(self._accel_data.z)
    )

  def _calculate_pitch_from_accelerometer(self, accel):
    if accel.z == 0.0:
      pitch = -np.sign(accel.x) * 90.0
    else:
      fract = -accel.x / accel.z
      pitch = np.arctan(fract) * AhrsConstants.RAD_TO_DEG
    
    return pitch

  def _store_imu_data(self, imu_data):
    self._mag_data.x = np.roll(self._mag_data.x, 1)
    self._mag_data.y = np.roll(self._mag_data.y, 1)
    self._mag_data.z = np.roll(self._mag_data.z, 1)
    self._mag_data.x[0] = imu_data.magnetometer.x
    self._mag_data.y[0] = imu_data.magnetometer.y
    self._mag_data.z[0] = imu_data.magnetometer.z

    self._accel_data.x = np.roll(self._accel_data.x, 1)
    self._accel_data.y = np.roll(self._accel_data.y, 1)
    self._accel_data.z = np.roll(self._accel_data.z, 1)
    self._accel_data.x[0] = imu_data.accelerometer.x
    self._accel_data.y[0] = imu_data.accelerometer.y
    self._accel_data.z[0] = imu_data.accelerometer.z