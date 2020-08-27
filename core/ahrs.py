# Author: Hoani
#
# Attitude Heading and Reference System - two wheeled
#
# Takes accelerometer, gyroscopic and magnetometer data and determines
# Robot angle with respect to the ground and the robot's pitch angle
from core.types import imuData, vec3
import numpy as np
import copy


def unit_vector(vector):
    return vector / np.linalg.norm(vector)


#
# Some basic triginometry to calculate the angle between two vectos
#
def angle_2d_deg(primary, secondary):
    p_u = unit_vector(primary)
    s_u = unit_vector(secondary)
    abs_radians = np.arccos(np.clip(np.dot(p_u, s_u), -1.0, 1.0))
    polarity = np.sign(np.cross(p_u, s_u))

    if polarity != 0:  # happens when cross product = 0
        radians = abs_radians * polarity
    else:
        radians = abs_radians
    return radians * AhrsConstants.RAD_TO_DEG


#
# Trapezoidal integration with a delta t and a rate
#
def integrate_rates(delta_t, rates_prev, rates):
    rates_sum = rates_prev + rates
    delta_angles = vec3.Vec3(
        rates_sum.x * 0.5 * delta_t,
        rates_sum.y * 0.5 * delta_t,
        rates_sum.z * 0.5 * delta_t
    )
    return delta_angles


def add_angle(angle1, angle2):
    result = angle1 + angle2
    while(result > AhrsConstants.DEG_WRAP):
        result -= AhrsConstants.DEG_PER_REV
    while(result < -AhrsConstants.DEG_WRAP):
        result += AhrsConstants.DEG_PER_REV
    return result


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
        self.gyro_drift = vec3.Vec3()
        self.calibrated = False
        self.max_gyro = 0.0
        self.max_accel = 0.0


class AhrsTwoWheeled:
    # Modes are used to supply additional information for the robot to infer
    # parameters
    #   MODE_DYNAMIC:
    #     Used in situations when the robot is moving, the algorithms rely
    #     only on the gyros. This is because:
    #       * the accelerometers cannot be relied on as providing a reliable
    #           gravity vector without using quaternions
    #           (potential improvement here)
    #       * the magnetometer has low bandwidth and is susceptable to noise
    #         (esp with motors running)
    #   MODE_SMART:
    #     Same as mode dynamic for yaw
    #     Uses a simple sensor fusion algorithm for pitch:
    #       If the absolute acceleration value is outside of gravity
    #       (given a small margin of 0.2(m/s2)), yaw is calculated from gyro
    #       only, otherwise, the yaw is calculated from a combination of both
    #       in order to limit the drift caused by yaw
    #   MODE_STILL:
    #     Corrects for integration error in pitch calculation by using
    #       accelerometers
    #     Corrects for integration error in yaw calculation by using
    #       magnetometers
    #   MODE_CALIBRATE:
    #     Assumes the robot is completely still and will not move until after
    #       calibration is complete
    #     Switches to MODE_STILL with no updates to calibrated values if high
    #       accelerations or angular rates are detected
    #     When calibrate is complete, the yaw angle is set to zero.
    MODE_DYNAMIC = 0
    MODE_SMART = 1
    MODE_STILL = 2
    MODE_CALIBRATE = 3

    #
    # Set the class map up here
    # it should be consistent between objects
    #
    def __new__(cls, *args, **kwargs):
        cls._update_map = {
            AhrsTwoWheeled.MODE_DYNAMIC:    cls._update_dynamic,
            AhrsTwoWheeled.MODE_SMART:      cls._update_dynamic,
            AhrsTwoWheeled.MODE_STILL:      cls._update_still,
            AhrsTwoWheeled.MODE_CALIBRATE:  cls._update_calibrate
        }
        return super().__new__(cls, *args, **kwargs)

    #
    # Create an Ahrs object
    #
    def __init__(self):
        self._offsets = imuData.ImuData(
            vec3.Vec3(),
            vec3.Vec3(),
            vec3.Vec3()
        )
        self._last_data = None
        self.yaw = 0.0
        self.pitch = 0.0
        self._mode = AhrsTwoWheeled.MODE_DYNAMIC

        self._cal = CalibrationData()
        self._mag_data = vec3.Vec3(
            np.zeros(AhrsConstants.MAG_AVERAGE),
            np.zeros(AhrsConstants.MAG_AVERAGE),
            np.zeros(AhrsConstants.MAG_AVERAGE)
        )
        self._accel_data = vec3.Vec3(
            np.zeros(AhrsConstants.ACCEL_AVERAGE),
            np.zeros(AhrsConstants.ACCEL_AVERAGE),
            np.zeros(AhrsConstants.ACCEL_AVERAGE)
        )
        self._still_count = 0

    #
    # Update the AHRS with the latest IMU data
    #
    def update(self, delta_t, imu_data):
        self._store_imu_data(imu_data)
        AhrsTwoWheeled._update_map[self._mode](self, delta_t, imu_data)
        self._last_data = copy.deepcopy(imu_data)

    #
    # Update yaw and pitch based on gyroscope measurements alone
    # Assumes the system is too dynamic to reliably use accelerations
    # or magnetometer measurements to determine attitude
    #
    def _update_dynamic(self, delta_t, imu_data):
        if self._last_data is None:
            return

        self.pitch, self.yaw = self._calculate_dynamic(delta_t, imu_data)

    #
    # Calculate the gyroscope based pitch and yaw
    #
    def _calculate_dynamic(self, delta_t, imu_data):
        rates_b = (imu_data.gyroscope - self._cal.gyro_drift)
        prev_rates_b = self._last_data.gyroscope - self._cal.gyro_drift
        R_b2l = self._rotation_b2l()

        delta_angle_b = integrate_rates(delta_t, prev_rates_b, rates_b).array()

        _, delta_pitch, delta_yaw = R_b2l.dot(delta_angle_b)

        yaw = add_angle(self.yaw, delta_yaw)
        pitch = add_angle(self.pitch, delta_pitch)
        return (pitch, yaw)

    #
    # Weight based filter which will fuse still model and dynamic models together
    #
    def _update_smart(self, delta_t, imu_data):
        still_pitch, weight = self._calculate_smart_still(delta_t, imu_data)
        dyn_pitch, dyn_yaw = self._calculate_dynamic(delta_t, imu_data)

        self.yaw = dyn_yaw
        self.pitch = still_pitch * weight + dyn_pitch * (1 - weight)

    #
    # Calculate weights as well as the still pitch vector
    # We don't care about yaw in this mode
    #
    def _calculate_smart_still(self, delta_t, imu_data):
        # Trust requirement 1: gyros are relatively still
        gyros = imu_data.gyroscope.array()
        gyro_lower, gyro_upper = 0.4, 2.4
        wg = (np.mean(gyros) + np.std(gyros) - gyro_lower)
        wg = wg / (gyro_upper - gyro_lower)

        accel_b = vec3.Vec3(
            np.mean(self._accel_data.x),
            np.mean(self._accel_data.y),
            np.mean(self._accel_data.z)
        )

        norm_accel_b = np.norm(accel_b.array())
        accel_lower, accel_upper = 0.5, 1.0
        wa = (norm_accel_b - AhrsConstants.DEFAULT_GRAVITY) - accel_lower
        wa = wa / (accel_upper - accel_lower)

        weight1 = 1.0 - max(min(wg, 1.0), 0.0)
        weight2 = 1.0 - max(min(wa, 1.0), 0.0)
        weight = weight1 * weight2

        pitch = self._calculate_pitch_from_gravity(accel_b)

        return pitch, weight

    #
    # Assumes the system is still (verifies with sensor measurements)
    # Aquires an accurate lock on attitude using accelerometers and
    # magnetometers
    #
    # Works better if calibrated
    #
    def _update_still(self, delta_t, imu_data):
        if np.linalg.norm(imu_data.gyroscope.array()) >= \
                AhrsConstants.CALIBRATION_GYRO_MAX:
            # Not still! Restart count
            self._still_count = 0
        else:
            self._still_count += 1

        if self._still_count >= AhrsConstants.ACCEL_AVERAGE:
            grav_b = vec3.Vec3(
                np.sum(self._accel_data.x),
                np.sum(self._accel_data.y),
                np.sum(self._accel_data.z)
            )
            self.pitch = self._calculate_pitch_from_gravity(grav_b)

        if self._cal.calibrated and \
                self._still_count >= AhrsConstants.MAG_AVERAGE:

            s = slice(0, 2)  # Only care about x and y coordinates here

            mag_b = np.array([
                np.average(self._mag_data.x),
                np.average(self._mag_data.y),
                np.average(self._mag_data.z)
            ])
            R_b2l = self._rotation_b2l()
            mag_l = R_b2l.dot(mag_b)
            mag_yaw = angle_2d_deg(
                mag_l[s],
                self._cal.magnetometer_zero.array()[s]
            )
            self.yaw = mag_yaw

    #
    # Continue to calibrate this bad boy
    #
    def _update_calibrate(self, delta_t, imu_data):
        if self._cal.data is None:
            self._cal.data = copy.deepcopy(imu_data)
        else:
            self._cal.data += imu_data

        self._cal.max_gyro = max(
            self._cal.max_gyro,
            np.linalg.norm(imu_data.gyroscope.array())
        )

        self._cal.max_accel = max(
            self._cal.max_accel,
            np.linalg.norm(imu_data.accelerometer.array())
        )

        self._cal.count += 1

    #
    # Return the Pitch and Yaw
    #
    def get(self):
        return AhrsAngles(self.pitch, self.yaw)

    #
    # Clear calibration data and start a new calibration run
    #
    def start_calibrate(self):
        # Clears all existing calibration data
        self._cal = CalibrationData()
        self._mode = AhrsTwoWheeled.MODE_CALIBRATE

    #
    # End and apply a calibration, moves the robot into still mode
    #
    def end_calibrate(self):
        # Check for bad calibration data
        accel_norm = np.linalg.norm(
            self._cal.data.accelerometer.array()
        )

        # Note: _cal.data is a sum
        accel_avg = accel_norm / self._cal.count

        bounds = AhrsConstants.GRAVITY_BOUNDS

        if accel_avg != np.clip(accel_avg, bounds[0], bounds[1]):
            return False  # Fell outside of the bounds

        # Check for too much movement during calibration
        if self._cal.max_accel > AhrsConstants.CALIBRATION_GRAVITY_MAX or \
                self._cal.max_gyro > AhrsConstants.CALIBRATION_GYRO_MAX:
            return False

        # We reset the yaw to 0.0 during calibration
        self.yaw = 0.0

        # The pitch angle is derived from the accelerometer
        self.pitch = self._calculate_pitch_from_gravity(
            self._cal.data.accelerometer
        )

        # Set calibrated gravity value
        self._cal.accelerometer_grav = accel_avg

        # Map calibrated magnetometer to local inertial frame
        R_b2l = self._rotation_b2l()
        mag_b = self._cal.data.magnetometer.array()

        mag_l = R_b2l.dot(mag_b)

        # Store the calibrated and mapped magnetometer values
        self._cal.magnetometer_zero = vec3.Vec3(mag_l[0], mag_l[1], mag_l[2])

        # Store the calibrated gyro offsets
        self._cal.gyro_drift = vec3.Vec3(
            self._cal.data.gyroscope.x / self._cal.count,
            self._cal.data.gyroscope.y / self._cal.count,
            self._cal.data.gyroscope.z / self._cal.count
        )

        # Change state
        self._mode = AhrsTwoWheeled.MODE_STILL
        self._cal.calibrated = True

        return True

    def set_calibrate(self):
        if self._mode != AhrsTwoWheeled.MODE_CALIBRATE:
            self.start_calibrate()

    def set_still(self):
        if self._mode == AhrsTwoWheeled.MODE_CALIBRATE:
            self.end_calibrate()

        self._mag_count = 0
        self._mode = AhrsTwoWheeled.MODE_STILL

    def set_dynamic(self):
        if self._mode == AhrsTwoWheeled.MODE_CALIBRATE:
            self.end_calibrate()

        self._mode = AhrsTwoWheeled.MODE_DYNAMIC

    def set_smart(self):
        if self._mode == AhrsTwoWheeled.MODE_CALIBRATE:
            self.end_calibrate()

        self._mode = AhrsTwoWheeled.MODE_SMART

    #
    # Computes the rotation matrix from body frame to a local inertial
    # reference frame
    #
    def _rotation_b2l(self):
        k = AhrsConstants.DEG_TO_RAD
        sin_pitch = np.sin(k * self.pitch)
        cos_pitch = np.cos(k * self.pitch)
        return np.array([
            [cos_pitch, 0.0, sin_pitch],
            [0.0, 1.0,       0.0],
            [-sin_pitch, 0.0, cos_pitch]
        ])

    #
    # Determine the pitch angle using a gravity vector
    #
    def _calculate_pitch_from_gravity(self, a_gravity):
        # Handle the dangerous div0 condition
        if a_gravity.z == 0.0:
            pitch = -np.sign(a_gravity.x) * 90.0
        else:
            # Actual math - we simplify a little bit by assuming the 2-wheeled
            # robot can't roll, therefore the y-component doesnt matter
            fract = -a_gravity.x / a_gravity.z
            pitch = np.arctan(fract) * AhrsConstants.RAD_TO_DEG

        return pitch

    #
    # Store IMU data for filtering
    #
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
