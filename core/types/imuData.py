from dataclasses import dataclass
from core.types.vec3 import Vec3


@dataclass
class ImuData:
    accelerometer: Vec3
    gyroscope: Vec3
    magnetometer: Vec3

    def __add__(self, other):
        return ImuData(
            self.accelerometer + other.accelerometer,
            self.gyroscope + other.gyroscope,
            self.magnetometer + other.magnetometer
        )

    def __sub__(self, other):
        return ImuData(
            self.accelerometer - other.accelerometer,
            self.gyroscope - other.gyroscope,
            self.magnetometer - other.magnetometer
        )


if "__main__" == __name__:
    print("Module not callable")
