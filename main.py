import sys
from machine import Pin, SPI, UART
# from cc1101 import CC1101
# from configuration import *
import utime
# from strobes import *
import uos
import uasyncio
from rfm69hcw.rfm69hcw import RFM69HCW

red_led = Pin(16, Pin.OUT) # extra
blue_led = Pin(2, Pin.OUT) # built-in

def blink(led, n=1):
    for i in range(n):
        led.value(0)
        utime.sleep_ms(200)
        led.value(1)
        utime.sleep_ms(200)


def init_spi():
    chip = sys.platform
    if chip == 'esp32':
        return SPI(1, baudrate=5000000, polarity=1, phase=1, sck=Pin(14), mosi=Pin(13), miso=Pin(12))
    elif chip == 'esp8266':
        return SPI(1, baudrate=5000000, polarity=1, phase=1)
    else:
        raise Exception('Cannot detect platform')

if sys.platform == 'esp32':
    cs = Pin(17, Pin.OUT)
    gdo0 = Pin(4, Pin.IN)
    gdo2 = Pin(16, Pin.IN)
else:
    cs = Pin(15, Pin.OUT)
    gdo0 = Pin(4, Pin.IN)
    gdo2 = Pin(5, Pin.IN)

spi = init_spi()

rst = Pin(27, Pin.OUT)

t = RFM69HCW(spi, cs, rst=rst)

t.reset()
