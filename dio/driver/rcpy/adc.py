import rcpy.adc as adc
from dio.driver.interfaces import AdcInterface


class Adc(AdcInterface):
    def __init__(self):
        # TODO: Should check if adc is avaliable and throw if it isn't
        pass

    def battery_voltage():
        return adc.battery.get_voltage()
