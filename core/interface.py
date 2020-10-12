from micropython import const
from core.helper import *
import utime

LOW = const(0x00)
HIGH = const(0x01)

class RadioInterface():

    def __init__(self, spi, cs, led_pin=None, led_source=True):
        
        self.led_source = led_source
        
        self.spi = spi
        self.cs = cs
        self.spi.init()
        if led_pin:
            led_pin.value(not led_source)
        self.led_pin = led_pin

    def _spi_read(self, address, length=2):
        self.cs.value(LOW)
        res = self.spi.read(length, address)
        self.cs.value(HIGH)
        return res

    def _spi_write(self, write_buf):
        read_buf = bytearray(len(write_buf))
        self.cs.value(LOW)
        self.spi.write_readinto(write_buf, read_buf)
        self.cs.value(HIGH)
        return read_buf

    def blink(self, n=1):
        if self.led_pin is None: return

        ON = self.led_source
        OFF = not self.led_source

        for _ in range(n):
            self.led_pin.value(ON)
            utime.sleep_ms(200)
            self.led_pin.value(OFF)
            utime.sleep_ms(200)

    def set_bits(self, register, change, start=0, length=8):
        current = read_bits(self.read(register))
        mask = str.format(BITS_F, 255)
        mask = mask[0:start] + '0'*length + mask[start+length:8]
        change_mask = str.format(BITS_F, 0)
        bits_change = str.format(BITS_F, change)[8-length:8]
        change_mask = change_mask[0:start] + bits_change + change_mask[start+length:8]
        change = current & int(mask, 2) | int(change_mask, 2)
        self.write(register, change)

    # def read(self, address, status_byte=False):
    #     # self.register_addr_space(address)
    #     # res = self._spi_read(address + SINGLE_READ)
    #     res = self._spi_read(address)
    #     if status_byte:
    #         return res[1], res[0]
    #     return res[1]

    # # def burst_read(self, address, n, status_byte=False):
    # #     # self.register_addr_space(address)
    # #     res = self._spi_read(address + BURST_READ, length=n+1)
    # #     if status_byte:
    # #         return res[1:], res[0]
    # #     return res[1:]

    # def write(self, address, byte):
    #     address = address + 0x80
    #     # self.register_addr_space(address)
    #     write_buf = bytearray([address, byte])
    #     return self._spi_write(write_buf)