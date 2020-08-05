from micropython import const
from machine import Pin
from time import sleep
from core.helper import *
from core.interface import RadioInterface
from .registers import *
import utime

PACKET_LENGTH_CONF = {
    0: 'FIXED',
    1: 'VARIABLE',
    2: 'INFINITE'
}

SINGLE_READ = const(0x00)
SINGLE_WRITE = const(0x80)


class RFM69HCW(RadioInterface):

    def __init__(self, spi, cs, rst=None, endian='big', xosc=32000000):
        
        super(RFM69HCW, self).__init__(spi, cs)

        self.FREQ_XOSC = xosc
        self.endian = endian
        self.rst = rst
        if rst:
            rst.value(0)

    def read(self, address):
         # first byte is empty
        return self._spi_read(address)[1]

    def write(self, address, byte):
        write_buf = bytearray([address + SINGLE_WRITE, byte])
        self._spi_write(write_buf)

    def reset(self):
        if not self.rst:
            print('no reset pin')
            return
        self.rst.value(1)
        utime.sleep_ms(5)
        self.rst.value(0)
        utime.sleep_ms(5)


    # RegFifo 0x00

    # RegOpMode 0x01

    def get_sequencer_off(self):
        res = read_bits(self.read(OPMODE), 0x00, 0x01)
        return res

    def set_sequencer_off(self, value):
        self.set_bits(OPMODE, value, 0x00, 0x01)

    def get_listen_on(self):
        res = read_bits(self.read(OPMODE), 0x01, 0x01)
        return res

    def set_listen_on(self, value):
        self.set_bits(OPMODE, value, 0x01, 0x01)

    def get_mode(self):
        res = read_bits(self.read(OPMODE), 0x03, 0x02)
        return res

    def set_mode(self, value):
        self.set_bits(OPMODE, value, 0x03, 0x02)

    # RegDataModul 0x02

    # RegBitrateMsb 0x03

    # RegBitrateMsb 0x04

    # RegPacketConfig1 0x37

    def get_packet_length_conf(self):
        res = read_bits(self.read(PACKETCONFIG1), 0x00, 0x01)
        return PACKET_LENGTH_CONF[res]

    def set_packet_length_conf(self, pkt_len):
        codes = reverse(PACKET_LENGTH_CONF)
        self.set_bits(PACKETCONFIG1, codes[pkt_len], 0x00, 0x01)
