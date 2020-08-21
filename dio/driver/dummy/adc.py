import random
from dio.driver.interfaces import AdcInterface


def noise(size):
    return size/2.0 - size*random.random()


class Adc(AdcInterface):
    def __init__(self):
        pass

    def battery_voltage():
        return 7.8 + noise(0.1)
