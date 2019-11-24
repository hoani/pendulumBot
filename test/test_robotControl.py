from source.bot import robotControl
from test.fake.bot import motorPairFake

class TestControlBasic:

  def test_initialization(self):
    wheels = motorPairFake.MotorPair();
    self.control = robotControl.RobotControl(wheels) 
    assert(self.control != None)
