from dataclasses import dataclass
import numpy as np


@dataclass
class Vec3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def __add__(self, other):
        return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other):
        return Vec3(self.x - other.x, self.y - other.y, self.z - other.z)

    def array(self):
        return np.array([self.x, self.y, self.z])


if "__main__" == __name__:
    print("Module not callable")
