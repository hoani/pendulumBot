import rcpy.adc as adc


def battery_voltage():
  return adc.battery.get_voltage()
  