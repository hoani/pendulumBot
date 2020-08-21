from app.control import pendulum
from app.test.fake.bot import ahrsFake
from app.test.fake.control import pidFake
from app.test.fake.bot import motorPairFake


class TestPendulumBasic:

    def test_initialization(self):
        ctrl = pendulum.PendulumPid()
        assert(ctrl is not None)

    def test_initializationWithFakes(self):
        pid = pidFake.PidFake()
        ahrs = ahrsFake.AhrsTwoWheeledFake()
        ctrl = pendulum.PendulumPid(pid, ahrs, 5.0)
        assert(ctrl is not None)


class TestPendulumSetLimits:

    def setup_method(self):
        self.pid = pidFake.PidFake()
        self.ahrs = ahrsFake.AhrsTwoWheeledFake()
        self.ctrl = pendulum.PendulumPid(self.pid, self.ahrs, 5.0)

    def test_set_limit(self):
        self.ctrl.set_limit(10.0)
        assert(self.ctrl.limit == 10.0)

    def test_set_negative_limit(self):
        self.ctrl.set_limit(-20.0)
        assert(self.ctrl.limit == 20.0)

    def test_set_text_limit(self):
        self.ctrl.set_limit("15.0")
        assert(self.ctrl.limit == 15.0)

    def test_set_nonsense_limit(self):
        self.ctrl.set_limit("Nonsense")
        assert(self.ctrl.limit == 5.0)

    def test_set_integer_limit(self):
        self.ctrl.set_limit(7)
        assert(self.ctrl.limit == 7.0)


class TestPendulumSetSetpoints:

    def setup_method(self):
        self.pid = pidFake.PidFake(setpoint=0.0)
        self.ahrs = ahrsFake.AhrsTwoWheeledFake()
        self.ctrl = pendulum.PendulumPid(self.pid, self.ahrs, 5.0)

    def test_set_setpoint(self):
        self.ctrl.set_setpoint(10.0)
        assert(self.pid.setpoint == 10.0)

    def test_set_negative_setpoint(self):
        self.ctrl.set_setpoint(-20.0)
        assert(self.pid.setpoint == -20.0)

    def test_set_text_setpoint(self):
        self.ctrl.set_setpoint("15.0")
        assert(self.pid.setpoint == 15.0)

    def test_set_nonsense_setpoint(self):
        self.ctrl.set_setpoint("Nonsense")
        assert(self.pid.setpoint == 0.0)

    def test_set_integer_setpoint(self):
        self.ctrl.set_setpoint(7)
        assert(self.pid.setpoint == 7.0)


class TestPendulumSetGains:

    def setup_method(self):
        self.pid = pidFake.PidFake(kp=1.0, ki=2.0, kd=3.0)
        self.ahrs = ahrsFake.AhrsTwoWheeledFake()
        self.ctrl = pendulum.PendulumPid(self.pid, self.ahrs, 5.0)

    def test_set_gains(self):
        self.ctrl.set_kp(10.0)
        self.ctrl.set_ki(20.0)
        self.ctrl.set_kd(30.0)
        assert(self.pid.kp == 10.0)
        assert(self.pid.ki == 20.0)
        assert(self.pid.kd == 30.0)

    def test_set_negative_gains(self):
        self.ctrl.set_kp(-1.0)
        self.ctrl.set_ki(-2.0)
        self.ctrl.set_kd(-3.0)
        assert(self.pid.kp == -1.0)
        assert(self.pid.ki == -2.0)
        assert(self.pid.kd == -3.0)

    def test_set_text_gains(self):
        self.ctrl.set_kp("10.0")
        self.ctrl.set_ki("20.0")
        self.ctrl.set_kd("30.0")
        assert(self.pid.kp == 10.0)
        assert(self.pid.ki == 20.0)
        assert(self.pid.kd == 30.0)

    def test_set_nonsense_gains(self):
        self.ctrl.set_kp("Nonsense")
        self.ctrl.set_ki("Nonsense")
        self.ctrl.set_kd("Nonsense")
        assert(self.pid.kp == 1.0)
        assert(self.pid.ki == 2.0)
        assert(self.pid.kd == 3.0)

    def test_set_integer_gains(self):
        self.ctrl.set_kp(10)
        self.ctrl.set_ki(-2)
        self.ctrl.set_kd(30)
        assert(self.pid.kp == 10.0)
        assert(self.pid.ki == -2.0)
        assert(self.pid.kd == 30.0)


class TestPendulumSetEnable:

    def setup_method(self):
        self.pid = pidFake.PidFake()
        self.ahrs = ahrsFake.AhrsTwoWheeledFake()
        self.ctrl = pendulum.PendulumPid(self.pid, self.ahrs, 5.0)

    def test_set_enable(self):
        self.ctrl.set_enable(True)
        assert(self.ctrl.enable is True)

    def test_set_disable(self):
        self.ctrl.set_enable(False)
        assert(self.ctrl.enable is False)

    def test_set_invalid_enable(self):
        self.ctrl.set_enable("True")
        self.ctrl.set_enable(1)
        self.ctrl.set_enable(1.0)
        assert(self.ctrl.enable is False)

    def test_resets_pid_on_enable(self):
        self.ctrl.set_enable(True)
        assert(self.pid.reset_called is True)


class TestPendulumUpdateSimple:

    def setup_method(self):
        self.wheels = motorPairFake.MotorPair()
        self.pid = pidFake.PidFake()
        self.ahrs = ahrsFake.AhrsTwoWheeledFake()
        self.ctrl = pendulum.PendulumPid(self.pid, self.ahrs, 5.0)

    def test_gets_ahrs_values(self):
        self.ctrl.set_enable(True)

        self.ctrl.update(self.wheels, 1)

        assert(self.ahrs.update_called is False)
        assert(self.ahrs.get_called is True)

    def test_updates_pid(self):
        self.ctrl.set_enable(True)

        self.ctrl.update(self.wheels, 1)

        assert(self.pid.update_called is True)

    def test_updates_pid_with_correct_args(self):
        self.ctrl.set_enable(True)
        self.ahrs.angles.pitch = -0.25

        self.ctrl.update(self.wheels, 500)

        assert(self.pid.update_args == (-0.25, 0.5))

    def test_update_returns_pid_output(self):
        self.ctrl.set_enable(True)
        self.pid.control_output = 0.5

        self.ctrl.update(self.wheels, 1)

        assert(self.wheels.last_call == motorPairFake.MotorPair.move)
        assert(self.wheels.speed == (0.5, 0.5))

    def test_update_clips_pid_output(self):
        self.ctrl.set_enable(True)
        self.pid.control_output = 1.5

        self.ctrl.update(self.wheels, 1)

        assert(self.wheels.last_call == motorPairFake.MotorPair.move)
        assert(self.wheels.speed == (1.0, 1.0))

    def test_update_clips_negative_pid_output(self):
        self.ctrl.set_enable(True)
        self.pid.control_output = -1.5

        self.ctrl.update(self.wheels, 1)

        assert(self.wheels.last_call == motorPairFake.MotorPair.move)
        assert(self.wheels.speed == (-1.0, -1.0))

    def test_no_updates_when_disabled(self):
        self.ctrl.set_enable(False)

        self.ctrl.update(self.wheels, 1)

        assert(self.ahrs.get_called is False)
        assert(self.pid.update_called is False)

    def test_returns_none_when_disabled(self):
        self.ctrl.set_enable(False)

        control_output = self.ctrl.update(self.wheels, 1)

        assert(control_output is None)


class TestSuspend:
    def setup_method(self):
        self.wheels = motorPairFake.MotorPair()
        self.pid = pidFake.PidFake()
        self.ahrs = ahrsFake.AhrsTwoWheeledFake()
        self.ctrl = pendulum.PendulumPid(self.pid, self.ahrs, 5.0)
        self.ctrl.set_enable(True)

    def test_suspend_simple(self):
        self.ctrl.set_setpoint(0.0)
        self.ahrs.set_angles(pitch=10.0, yaw=0.0)
        self.pid.set_control_output(1.0)

        self.ctrl.update(self.wheels, 1)

        assert(self.wheels.last_call == motorPairFake.MotorPair.stop)

    def test_suspend_negative(self):
        self.ctrl.set_setpoint(0.0)
        self.ahrs.set_angles(pitch=-10.0, yaw=0.0)
        self.pid.set_control_output(1.0)

        self.ctrl.update(self.wheels, 1)

        assert(self.wheels.last_call == motorPairFake.MotorPair.stop)

    def test_suspend_exact(self):
        self.ctrl.set_setpoint(0.0)
        self.ahrs.set_angles(pitch=5.0, yaw=0.0)
        self.pid.set_control_output(1.0)

        self.ctrl.update(self.wheels, 1)

        assert(self.wheels.last_call == motorPairFake.MotorPair.stop)

    def test_suspend_recovery(self):
        self.ctrl.set_setpoint(0.0)
        self.ahrs.set_angles(pitch=5.0, yaw=0.0)
        self.pid.set_control_output(1.0)

        self.ctrl.update(self.wheels, 1)

        self.ahrs.set_angles(pitch=0.0, yaw=0.0)
        self.ctrl.update(self.wheels, 1)

        assert(self.wheels.last_call == motorPairFake.MotorPair.move)
        assert(self.wheels.speed == (1.0, 1.0))
        assert(self.pid.reset_called)
