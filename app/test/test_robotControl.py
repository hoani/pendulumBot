from app.control import modes as robotControl
from app.test.fake.bot import motorPairFake


class TestControlBasic:

    def test_initialization(self):
        wheels = motorPairFake.MotorPair()
        self.control = robotControl.RobotControl(wheels)
        assert(self.control is not None)
