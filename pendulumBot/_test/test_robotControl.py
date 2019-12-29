from pendulumBot.bot import robotControl
from pendulumBot._test.fake.bot import motorPairFake

class TestControlBasic:

  def test_initialization(self):
    wheels = motorPairFake.MotorPair()
    self.control = robotControl.RobotControl(wheels) 
    assert(self.control != None)
