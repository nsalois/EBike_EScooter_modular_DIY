import board
import displayio
import adafruit_displayio_ssd1306

class Display(object):
    def __init__(self):

        displayio.release_displays()

        i2c = board.I2C()

        display_bus = displayio.I2CDisplay(i2c, device_address=0x3c)

        WIDTH = 128
        HEIGHT = 64
        BORDER = 0

        self._display = adafruit_displayio_ssd1306.SSD1306(display_bus, width=128, height=64)

    @property
    def display(self):
        return self._display