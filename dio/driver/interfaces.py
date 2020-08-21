from abc import ABC, abstractmethod


class AdcInterface(ABC):
    @abstractmethod
    def battery_voltage():
        pass


class ImuInterface(ABC):
    @abstractmethod
    def sample(self):
        pass


class MotorInterface(ABC):
    @abstractmethod
    def run(self, speed):
        pass

    @abstractmethod
    def stop(self):
        pass

    @abstractmethod
    def get_duty(self):
        pass


class ServoInterface(ABC):
    @abstractmethod
    def set_duty(self, duty):
        pass

    @abstractmethod
    def disable(self):
        pass

    @abstractmethod
    def get_duty(self):
        pass
