import digitalio

class PAS(object):
    """PAS sensor"""
    def __init__(self, pin):
        """Brake sensor
        :param ~microcontroller.Pin pin: IO pin used to read brake sensor
        """
        # configure IO input
        # # about pull up: the ESP32 internal pullups are weak and are not enough for the brake sensor
        self._pas = digitalio.DigitalInOut(pin)
        self._pas.direction = digitalio.Direction.INPUT
        self._pas.pull = digitalio.Pull.UP

    @property
    def value(self):
        return self._pas.value