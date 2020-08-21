from core.types import vec3, imuData
import random
from dio.driver.interfaces import ImuInterface


def noise(size):
    return size/2.0 - size*random.random()


class Imu(ImuInterface):
    def __init__(self):
        pass

    def sample(self):
        accelerometer = vec3.Vec3(
            0.0 + noise(0.8),
            0.0 + noise(0.6),
            9.8 + noise(0.8)
        )
        gyroscope = vec3.Vec3(
            0.0 + noise(1.0),
            0.0 + noise(1.0),
            0.0 + noise(1.2)
        )
        magnetometer = vec3.Vec3(
            10 + noise(5.0),
            20 + noise(4.0),
            -20 + noise(5.0)
        )
        return imuData.ImuData(accelerometer, gyroscope, magnetometer)


if __name__ == "__main__":
    print("This module is not executable")
