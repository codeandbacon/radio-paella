from micropython import const
from machine import Pin
from time import sleep
from core.helper import *
from core.interface import RadioInterface
from .registers import *
import utime

MODE = {
    0: 'SLEEP',
    1: 'STDBY',
    2: 'FS',
    3: 'TX',
    4: 'RX'
}

PACKET_LENGTH_CONF = {
    0: 'FIXED',
    1: 'VARIABLE',
    2: 'INFINITE'
}

MODULATION_TYPE = {
    0: 'FSK',
    1: 'OOK'
}

DC_FREE = {
    0: 'NONE',
    1: 'MANCHESTER',
    2: 'WHITENING'
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

    def burst_read(self, address, n):
        res = self._spi_read(address, length=n+1)
        return res[1:]

    def write(self, address, byte):
        write_buf = bytearray([address + SINGLE_WRITE, byte])
        self._spi_write(write_buf)

    def burst_write(self, address, data):
        write_buf = bytearray([address + SINGLE_WRITE]) + bytearray(data)
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

    def get_fifo(self):
        return self.read(FIFO)

    def set_fifo(self, data):
        self.write(FIFO, data)

    # RegOpMode 0x01

    def get_sequencer_off(self):
        return read_bits(self.read(OPMODE), 0x00, 0x01)

    def set_sequencer_off(self, value):
        self.set_bits(OPMODE, value, 0x00, 0x01)

    def get_listen_on(self):
        return read_bits(self.read(OPMODE), 0x01, 0x01)

    def set_listen_on(self, value):
        self.set_bits(OPMODE, value, 0x01, 0x01)

    def get_mode(self):
        res = read_bits(self.read(OPMODE), 0x03, 0x02)
        return MODE[res]

    def set_mode(self, value):
        codes = reverse(MODE)
        self.set_bits(OPMODE, codes[value], 0x03, 0x02)

    # RegDataModul 0x02

    def get_data_mode(self):
        res = read_bits(self.read(DATAMODUL), 0x00, 0x02)
        return res

    def set_data_mode(self, value):
        pass

    def get_modulation_type(self):
        res = read_bits(self.read(DATAMODUL), 0x02, 0x02)
        return MODULATION_TYPE[res]

    def set_modulation_type(self, value):
        codes = reverse(MODULATION_TYPE)
        self.set_bits(DATAMODUL, codes[value], 0x02, 0x02)

    def get_modulation_shaping(self):
        res = read_bits(self.read(DATAMODUL), 0x07, 0x02)
        return res

    # RegBitrateMsb 0x03
    # RegBitrateLsb 0x04

    def get_bitrate(self):
        bitrate = (read_bits(self.read(BITRATEMSB)) << 8) + read_bits(self.read(BITRATELSB))
        return self.FREQ_XOSC / bitrate

    def set_bitrate(self, value):
        value = round(self.FREQ_XOSC / value)
        bitrate = value.to_bytes(2, self.endian)
        self.burst_write(BITRATEMSB, bitrate)

    # RegRfrMsb 0x07
    # RegRfrMid 0x08
    # RegRfrLsb 0x09

    def get_frequency(self):
        f_step = self.FREQ_XOSC / (2**19)
        f_rf = int.from_bytes(self.burst_read(RFRMSB, 3), 3, self.endian)
        return f_step * f_rf

    def set_frequency(self, value):
        f_step = self.FREQ_XOSC / (2**19)
        fr = round(value/f_step)
        data = fr.to_bytes(3, self.endian)
        self.burst_write(RFRMSB, data)

    # RegIrqFlags2 0x28

    def get_fifo_not_empty(self):
        return read_bits(self.read(SYNCCONFIG), 0x01, 0x01)

    # RegPreambleMsb 0x2c
    # RegPreambleLsb 0x2d

    def get_preamble_size(self):
        return int.from_bytes(self.burst_read(PREAMBLEMSB, 2), 2, self.endian)

    def set_preamble_size(self, value):
        data = value.to_bytes(2, self.endian)
        self.burst_write(PREAMBLEMSB, data)

    # RegSyncConfig 0x2e

    def get_sync_on(self):
        return read_bits(self.read(SYNCCONFIG), 0x00, 0x01)

    def set_sync_on(self, value):
        self.set_bits(SYNCCONFIG, value, 0x00, 0x01)

    def get_sync_size(self):
        return read_bits(self.read(SYNCCONFIG), 0x02, 0x03)

    def set_sync_size(self, value):
        self.set_bits(SYNCCONFIG, value, 0x02, 0x03)  

    # RegPacketConfig1 0x37

    def get_packet_length_conf(self):
        res = read_bits(self.read(PACKETCONFIG1), 0x00, 0x01)
        return PACKET_LENGTH_CONF[res]

    def set_packet_length_conf(self, value):
        codes = reverse(PACKET_LENGTH_CONF)
        self.set_bits(PACKETCONFIG1, codes[value], 0x00, 0x01)

    def get_dc_free(self):
        res = read_bits(self.read(PACKETCONFIG1), 0x01, 0x02)
        return DC_FREE[res]

    def set_dc_free(self, value):
        codes = reverse(DC_FREE)
        self.set_bits(PACKETCONFIG1, codes[value], 0x01, 0x02)

    def get_crc_on(self):
        return read_bits(self.read(PACKETCONFIG1), 0x03, 0x01)

    def set_crc_on(self, value):
        self.set_bits(PACKETCONFIG1, value, 0x03, 0x01)

    def get_address_filtering(self):
        return read_bits(self.read(PACKETCONFIG1), 0x05, 0x02)

    def set_address_filtering(self, value):
        self.set_bits(PACKETCONFIG1, value, 0x05, 0x02)
