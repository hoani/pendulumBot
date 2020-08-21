from core import ahrs
from core.types import vec3, imuData
import numpy as np


def abs_diff(val1, val2):
    return abs(val1 - val2)


class TestAhrsBasic:

    def test_initialization(self):
        _ahrs = ahrs.AhrsTwoWheeled()
        assert(_ahrs is not None)

    def test_initial_values(self):
        _ahrs = ahrs.AhrsTwoWheeled()
        values = _ahrs.get()
        assert(isinstance(values.pitch, float))
        assert(isinstance(values.yaw, float))
        assert(values.pitch == 0.0)
        assert(values.yaw == 0.0)


class TestAhrsDynamicUpdates:

    def setup_method(self):
        self.ahrs = ahrs.AhrsTwoWheeled()
        self.imu_data = imuData.ImuData(
            vec3.Vec3(0.0, 0.0, ahrs.AhrsConstants.DEFAULT_GRAVITY),
            vec3.Vec3(0.0, 0.0, 0.0),
            vec3.Vec3(40.0, 0.0, 0.0)
        )

    def test_simple_update(self):
        self.ahrs.update(1.0, self.imu_data)
        values = self.ahrs.get()
        assert(values.yaw == 0.0)
        assert(values.pitch == 0.0)

    def test_simple_turn(self):
        self.imu_data.gyroscope.z = 2.0
        self.ahrs.update(1.0, self.imu_data)
        values = self.ahrs.get()
        assert(values.yaw == 2.0)

    def test_quick_turn(self):
        self.imu_data.gyroscope.z = 10.0
        self.ahrs.update(0.25, self.imu_data)
        values = self.ahrs.get()
        assert(values.yaw == 2.5)

    def test_cumulative_turn(self):
        self.imu_data.gyroscope.z = 10.0
        for i in range(8):
            self.ahrs.update(0.125, self.imu_data)
        values = self.ahrs.get()
        assert(values.yaw == 10.0)

    def test_negative_turn(self):
        self.imu_data.gyroscope.z = -10.0
        self.ahrs.update(1.0, self.imu_data)
        values = self.ahrs.get()
        assert(values.yaw == -10.0)

    def test_wrap_around_turn(self):
        self.imu_data.gyroscope.z = 200.0
        self.ahrs.update(1.0, self.imu_data)
        values = self.ahrs.get()
        assert(values.yaw == -160.0)

    def test_wrap_around_neg_turn(self):
        self.imu_data.gyroscope.z = -200.0
        self.ahrs.update(1.0, self.imu_data)
        values = self.ahrs.get()
        assert(values.yaw == 160.0)

    def test_wrap_around_multi_turn(self):
        self.imu_data.gyroscope.z = 3660.0
        self.ahrs.update(1.0, self.imu_data)
        values = self.ahrs.get()
        assert(values.yaw == 60.0)

    def test_simple_tilt(self):
        self.imu_data.gyroscope.y = 2.0
        self.ahrs.update(1.0, self.imu_data)
        values = self.ahrs.get()
        assert(values.pitch == 2.0)

    def test_yaw_orthogonality_x(self):
        self.imu_data.gyroscope.x = 2.0
        self.ahrs.update(1.0, self.imu_data)
        values = self.ahrs.get()
        assert(values.yaw == 0.0)

    def test_yaw_orthogonality_z(self):
        self.ahrs.pitch = 90.0
        self.imu_data.gyroscope.z = 2.0
        self.ahrs.update(1.0, self.imu_data)
        values = self.ahrs.get()
        assert(abs_diff(values.yaw, 0.0) < 0.0001)

    def test_yaw_90_deg(self):
        self.ahrs.pitch = 90.0
        self.imu_data.gyroscope.x = 2.0
        self.ahrs.update(1.0, self.imu_data)
        values = self.ahrs.get()
        assert(abs_diff(values.yaw, -2.0) < 0.0001)

    def test_yaw_30_deg(self):
        self.ahrs.pitch = 30.0
        self.imu_data.gyroscope.x = 10.0
        self.ahrs.update(1.0, self.imu_data)
        values = self.ahrs.get()
        assert(abs_diff(values.yaw, -5.0) < 0.0001)

    def test_remove_offsets(self):
        self.ahrs.start_calibrate()
        calibrate_data = imuData.ImuData(
            vec3.Vec3(0.0, 0.0, ahrs.AhrsConstants.DEFAULT_GRAVITY),
            vec3.Vec3(0.0, 0.0, 0.5),
            vec3.Vec3()
        )

        for i in range(20):
            self.ahrs.update(0.1, calibrate_data)

        calibrated = self.ahrs.end_calibrate()

        assert(abs_diff(self.ahrs._cal.gyro_drift.z,  0.5) < 0.001)
        assert(calibrated)

        self.ahrs.set_dynamic()

        self.ahrs.pitch = 0.0
        self.imu_data.gyroscope.z = 5.5
        self.ahrs.update(1.0, self.imu_data)
        values = self.ahrs.get()
        assert(abs_diff(values.yaw, 5.0) < 0.0001)


class TestAhrsCalibrate:
    def setup_method(self):
        self.ahrs = ahrs.AhrsTwoWheeled()
        self.imu_data = imuData.ImuData(
            vec3.Vec3(0.0, 0.0, ahrs.AhrsConstants.DEFAULT_GRAVITY),
            vec3.Vec3(0.0, 0.0, 0.0),
            vec3.Vec3(40.0, 0.0, 0.0)
        )

    def test_yaw_is_zero_after_calibration(self):
        self.ahrs.yaw = 12.0
        self.ahrs.start_calibrate()

        for i in range(20):
            self.ahrs.update(0.1, self.imu_data)
        calibrated = self.ahrs.end_calibrate()

        values = self.ahrs.get()
        assert(values.yaw == 0.0)
        assert(calibrated)

    def test_calibration_perfect(self):
        # Calibrate magnetometer to 0 deg = x axis
        self.ahrs.start_calibrate()
        for i in range(20):
            self.ahrs.update(0.1, self.imu_data)
        calibrated = self.ahrs.end_calibrate()
        # Change magnetic field to be on y-axis (-90 deg)
        self.imu_data.magnetometer.x = 0.0
        self.imu_data.magnetometer.y = 40.0
        for i in range(20):
            self.ahrs.update(0.1, self.imu_data)

        values = self.ahrs.get()
        assert(abs_diff(values.yaw, -90.0) < 0.0001)
        assert(calibrated)

    def test_calibration_45_deg(self):
        # Calibrate magnetometer to 0 deg = x axis
        self.ahrs.start_calibrate()
        for i in range(20):
            self.ahrs.update(0.1, self.imu_data)
        calibrated = self.ahrs.end_calibrate()
        # Change magnetic field to be on x and y-axis (-45 deg)
        self.imu_data.magnetometer.x = 30.0
        self.imu_data.magnetometer.y = 30.0
        for i in range(20):
            self.ahrs.update(0.1, self.imu_data)

        values = self.ahrs.get()
        assert(abs_diff(values.yaw, -45.0) < 0.0001)
        assert(calibrated)

    def test_calibration_positive_yaw(self):
        # Calibrate magnetometer to 0 deg = x axis
        self.ahrs.start_calibrate()
        for i in range(20):
            self.ahrs.update(0.1, self.imu_data)
        calibrated = self.ahrs.end_calibrate()
        # Change magnetic field to be on y-axis (90 deg)
        self.imu_data.magnetometer.x = 0.0
        self.imu_data.magnetometer.y = -40.0
        for i in range(20):
            self.ahrs.update(0.1, self.imu_data)

        values = self.ahrs.get()
        assert(abs_diff(values.yaw, 90.0) < 0.0001)
        assert(calibrated)

    def test_calibration_180_yaw(self):
        # Calibrate magnetometer to 0 deg = x axis
        self.ahrs.start_calibrate()
        for i in range(20):
            self.ahrs.update(0.1, self.imu_data)
        calibrated = self.ahrs.end_calibrate()
        # Change magnetic field to be on y-axis (90 deg)
        self.imu_data.magnetometer.x = -40.0
        self.imu_data.magnetometer.y = 0.0
        for i in range(20):
            self.ahrs.update(0.1, self.imu_data)

        values = self.ahrs.get()
        assert(abs_diff(values.yaw, 180.0) < 0.0001)
        assert(calibrated)

    def test_calibration_magnetometer_averaging(self):
        # Calibrate magnetometer to 0 deg = x axis
        self.ahrs.start_calibrate()
        for i in range(-5, 5):
            self.imu_data.magnetometer.x = 40.0 + float(i) + 0.5
            self.imu_data.magnetometer.y = 0.0 - float(i) - 0.5
            self.imu_data.magnetometer.z = 0.0 + float(i) + 0.5
            self.ahrs.update(0.1, self.imu_data)
        calibrated = self.ahrs.end_calibrate()
        # Change magnetic field to be on y-axis (90 deg)
        self.imu_data.magnetometer.x = 0.0
        self.imu_data.magnetometer.y = 40.0
        for i in range(20):
            self.ahrs.update(0.1, self.imu_data)

        values = self.ahrs.get()
        assert(abs_diff(values.yaw, -90.0) < 0.0001)
        assert(calibrated)

    def test_calibration_magnetometer_projection_no_pitch(self):
        # Calibrate magnetometer to 0 deg => y = f(x) = x
        self.ahrs.start_calibrate()
        for i in range(20):
            self.imu_data.magnetometer.x = 40.0
            self.imu_data.magnetometer.y = 40.0
            self.imu_data.magnetometer.z = 20.0
            self.ahrs.update(0.1, self.imu_data)
        calibrated = self.ahrs.end_calibrate()
        # Change magnetic field to be 45 deg on the x-y plane
        self.imu_data.magnetometer.x = 0.0
        self.imu_data.magnetometer.y = 70.0
        self.imu_data.magnetometer.z = 25.0
        for i in range(20):
            self.ahrs.update(0.1, self.imu_data)

        values = self.ahrs.get()
        assert(abs_diff(values.yaw, -45.0) < 0.0001)
        assert(calibrated)

    def test_calibration_magnetometer_projection_90_pitch(self):
        # Calibrate magnetometer to 0 deg => y = f(x) = x = z
        self.ahrs.start_calibrate()
        for i in range(20):
            self.imu_data.magnetometer.x = 40.0
            self.imu_data.magnetometer.y = 40.0
            self.imu_data.magnetometer.z = 40.0
            self.ahrs.update(0.1, self.imu_data)
        calibrated = self.ahrs.end_calibrate()
        self.ahrs.pitch = 90.0
        # Change magnetic field to be 45 deg on the x-y plane
        # Note: X values shouldn't matter as it maps to global z
        self.imu_data.magnetometer.x = -30.0
        self.imu_data.magnetometer.y = 40.0
        self.imu_data.magnetometer.z = 0.0
        # rotate by 90
        self.imu_data.accelerometer.z = 0.0
        self.imu_data.accelerometer.x = -ahrs.AhrsConstants.DEFAULT_GRAVITY
        for i in range(20):
            self.ahrs.update(0.1, self.imu_data)

        values = self.ahrs.get()
        assert(abs_diff(values.yaw, -45.0) < 0.0001)
        assert(calibrated)

    def test_calibration_magnetometer_at_45_pitch(self):
        # Calibrate magnetometer to 0 deg => y = f(x) = x = z
        self.ahrs.pitch = 45.0
        self.imu_data.accelerometer.z = ahrs.AhrsConstants.DEFAULT_GRAVITY * \
            np.sqrt(0.5)
        self.imu_data.accelerometer.x = -self.imu_data.accelerometer.z
        self.ahrs.start_calibrate()

        self.imu_data.magnetometer.x = 00.0
        self.imu_data.magnetometer.y = 40.0
        self.imu_data.magnetometer.z = 40.0*np.sqrt(2.0)
        for i in range(20):
            self.ahrs.update(0.1, self.imu_data)

        calibrated = self.ahrs.end_calibrate()
        self.ahrs.pitch = 0.0
        # Change magnetic field to be 45 deg on the x-y plane
        # Note: X values shouldn't matter as it maps to global z
        self.imu_data.magnetometer.x = 0.0
        self.imu_data.magnetometer.y = 40.0
        self.imu_data.magnetometer.z = -300.0
        # rotate by 90
        self.imu_data.accelerometer.z = ahrs.AhrsConstants.DEFAULT_GRAVITY
        self.imu_data.accelerometer.x = 0.0
        for i in range(20):
            self.ahrs.update(0.1, self.imu_data)

        values = self.ahrs.get()
        assert(abs_diff(values.yaw, -45.0) < 0.0001)
        assert(calibrated)

    def test_calibration_acceleration(self):
        self.imu_data.accelerometer.x = 10.0 * \
            np.sin(20.0 * ahrs.AhrsConstants.DEG_TO_RAD)
        self.imu_data.accelerometer.y = 0.0
        self.imu_data.accelerometer.z = 10.0 * \
            np.cos(20.0 * ahrs.AhrsConstants.DEG_TO_RAD)
        self.ahrs.start_calibrate()

        for i in range(10):
            self.ahrs.update(0.1, self.imu_data)

        calibrated = self.ahrs.end_calibrate()

        assert(abs_diff(self.ahrs._cal.accelerometer_grav, 10.0) < 0.0001)
        assert(calibrated)

    def test_calibration_average_accel_fail_high(self):
        self.imu_data.accelerometer.x = 10.5
        self.imu_data.accelerometer.y = 0.0
        self.imu_data.accelerometer.z = 0.0
        self.ahrs.start_calibrate()

        for i in range(10):
            self.ahrs.update(0.1, self.imu_data)

        calibrated = self.ahrs.end_calibrate()

        assert(not calibrated)

    def test_calibration_average_accel_fail_low(self):
        self.imu_data.accelerometer.x = 9.0
        self.imu_data.accelerometer.y = 0.0
        self.imu_data.accelerometer.z = 0.0
        self.ahrs.start_calibrate()

        for i in range(10):
            self.ahrs.update(0.1, self.imu_data)

        calibrated = self.ahrs.end_calibrate()

        assert(calibrated is False)

    def test_calibration_max_accel_fail_high(self):
        self.imu_data.accelerometer.x = 9.8
        self.imu_data.accelerometer.y = 0.0
        self.imu_data.accelerometer.z = 0.0
        self.ahrs.start_calibrate()

        for i in range(5):
            self.ahrs.update(0.1, self.imu_data)

        self.imu_data.accelerometer.x = 11.4
        self.ahrs.update(0.1, self.imu_data)
        self.imu_data.accelerometer.x = 9.8

        for i in range(5):
            self.ahrs.update(0.1, self.imu_data)

        calibrated = self.ahrs.end_calibrate()

        assert(not calibrated)

    def test_calibration_max_gyro_fail_high(self):
        self.ahrs.start_calibrate()

        for i in range(5):
            self.ahrs.update(0.1, self.imu_data)

        self.imu_data.gyroscope.x = 11.4
        self.ahrs.update(0.1, self.imu_data)
        self.imu_data.gyroscope.x = 0.0

        for i in range(5):
            self.ahrs.update(0.1, self.imu_data)

        calibrated = self.ahrs.end_calibrate()

        assert(not calibrated)

    def test_calibration_gyro_drift_avg(self):
        self.ahrs.start_calibrate()
        self.imu_data.gyroscope.x = 0.4
        self.imu_data.gyroscope.y = -0.3
        self.imu_data.gyroscope.z = 0.0

        for i in range(-5, 6):
            self.imu_data.gyroscope.z = float(i)/10.0
            self.ahrs.update(0.1, self.imu_data)

        calibrated = self.ahrs.end_calibrate()

        assert(abs_diff(self.ahrs._cal.gyro_drift.x,  0.4) < 0.001)
        assert(abs_diff(self.ahrs._cal.gyro_drift.y, -0.3) < 0.001)
        assert(abs_diff(self.ahrs._cal.gyro_drift.z,  0.0) < 0.001)
        assert(calibrated)


class TestAhrsStill:
    def setup_method(self):
        self.ahrs = ahrs.AhrsTwoWheeled()
        self.imu_data = imuData.ImuData(
            vec3.Vec3(0.0, 0.0, ahrs.AhrsConstants.DEFAULT_GRAVITY),
            vec3.Vec3(0.0, 0.0, 0.0),
            vec3.Vec3(40.0, 0.0, 0.0)
        )
        self.ahrs.start_calibrate()
        for i in range(10):
            self.ahrs.update(0.1, self.imu_data)
        self.ahrs.end_calibrate()

    def test_gyro_ignored_when_still(self):
        self.imu_data.gyroscope.z = 3.0
        self.ahrs.set_still()
        self.ahrs.update(1.0, self.imu_data)
        values = self.ahrs.get()
        assert(values.yaw == 0.0)

    def test_mag_average(self):
        self.imu_data.magnetometer.y = -40.0
        self.ahrs.set_still()

        self.ahrs.update(0.1, self.imu_data)

        for i in range(-5, 5):
            self.imu_data.magnetometer.x = float(i) + 0.5
            self.ahrs.update(0.1, self.imu_data)

        values = self.ahrs.get()
        assert(values.yaw == 90.0)

    def test_accel_average(self):
        self.imu_data.accelerometer.x = -9.81 * \
            np.sin(20.0 * ahrs.AhrsConstants.DEG_TO_RAD)
        self.imu_data.accelerometer.y = 0.0
        self.imu_data.accelerometer.z = 9.81 * \
            np.cos(20.0 * ahrs.AhrsConstants.DEG_TO_RAD)

        self.ahrs.set_still()

        self.ahrs.update(0.1, self.imu_data)

        x_mean = self.imu_data.accelerometer.x
        for i in range(-5, 5):
            self.imu_data.accelerometer.x = x_mean + float(i)/10.0 + 0.05
            self.ahrs.update(0.1, self.imu_data)

        values = self.ahrs.get()
        assert(abs_diff(values.pitch, 20.0) < 0.001)

    def test_movement_resets_averaging(self):
        self.imu_data.accelerometer.x = -9.81 * \
            np.sin(20.0 * ahrs.AhrsConstants.DEG_TO_RAD)
        self.imu_data.accelerometer.y = 0.0
        self.imu_data.accelerometer.z = 9.81 * \
            np.cos(20.0 * ahrs.AhrsConstants.DEG_TO_RAD)

        self.ahrs.set_still()

        self.ahrs.update(0.1, self.imu_data)

        x_mean = self.imu_data.accelerometer.x
        for i in range(-5, 0):
            self.imu_data.accelerometer.x = x_mean + float(i)/10.0 + 0.05
            self.ahrs.update(0.1, self.imu_data)

        # Movement - should prevent us from averaging
        self.imu_data.gyroscope.x = ahrs.AhrsConstants.CALIBRATION_GYRO_MAX
        self.imu_data.accelerometer.x = x_mean + 0.05
        self.ahrs.update(0.1, self.imu_data)

        for i in range(1, 5):
            self.imu_data.accelerometer.x = x_mean + float(i)/10.0 + 0.05
            self.ahrs.update(0.1, self.imu_data)

        values = self.ahrs.get()
        assert(abs_diff(values.pitch, 20.0) > 0.001)


class TestAhrsHelpers:
    def test_angle_2d_deg(self):
        v1 = np.array([1, 0])
        v2 = np.array([0, 1])
        v3 = np.array([1, 1])

        assert(abs_diff(ahrs.angle_2d_deg(v1, v2), 90.0) < 1e-12)
        assert(abs_diff(ahrs.angle_2d_deg(v2, v1), -90.0) < 1e-12)
        assert(abs_diff(ahrs.angle_2d_deg(v1, v1), 0.0) < 1e-12)
        assert(abs_diff(ahrs.angle_2d_deg(v1, -v1),  180.0) < 1e-12)
        assert(abs_diff(ahrs.angle_2d_deg(v1, v3), 45.0) < 1e-12)
        assert(abs_diff(ahrs.angle_2d_deg(v2, v3), -45.0) < 1e-12)
        assert(abs_diff(ahrs.angle_2d_deg(-v1, v3), -135.0) < 1e-12)
        assert(abs_diff(ahrs.angle_2d_deg(v3, -v1), 135.0) < 1e-12)
