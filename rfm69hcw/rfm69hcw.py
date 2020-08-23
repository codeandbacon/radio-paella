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
        addr = FIFO
        buf = bytearray([addr]) + data
        return self._spi_write(buf)        

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
        res = read_bits(self.read(OPMODE), 0x03, 0x03)
        return MODE[res]

    def set_mode(self, value):
        codes = reverse(MODE)
        self.set_bits(OPMODE, codes[value], 0x03, 0x03)

    # RegDataModul 0x02

    def get_data_mode(self):
        return read_bits(self.read(DATAMODUL), 0x01, 0x02)

    def set_data_mode(self, value):
        self.set_bits(DATAMODUL, value, 0x01, 0x02)

    def get_modulation_type(self):
        res = read_bits(self.read(DATAMODUL), 0x03, 0x02)
        return MODULATION_TYPE[res]

    def set_modulation_type(self, value):
        codes = reverse(MODULATION_TYPE)
        self.set_bits(DATAMODUL, codes[value], 0x03, 0x02)

    def get_modulation_shaping(self):
        return read_bits(self.read(DATAMODUL), 0x06, 0x02)

    def set_modulation_shaping(self, value):
        self.set_bits(DATAMODUL, value, 0x06, 0x02)

    # RegBitrateMsb 0x03
    # RegBitrateLsb 0x04

    def get_bitrate(self):
        bitrate = (read_bits(self.read(BITRATEMSB)) << 8) + read_bits(self.read(BITRATELSB))
        return self.FREQ_XOSC / bitrate

    def set_bitrate(self, value):
        value = round(self.FREQ_XOSC / value)
        bitrate = value.to_bytes(2, self.endian)
        self.burst_write(BITRATEMSB, bitrate)

    # RegFdevMsb 0x05
    # RegFdevMsb 0x06

    def get_deviation(self):
        f_step = self.FREQ_XOSC / (2**19)
        dev = int.from_bytes(self.burst_read(FDEVMSB, 2), 2, self.endian)
        return dev * f_step

    def set_deviation(self, value):
        f_step = self.FREQ_XOSC / (2**19)
        dev = int(value/f_step).to_bytes(2, self.endian)
        self.burst_write(FDEVMSB, dev)

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

    # RegVersion 0x10

    def get_version(self):
        value = '{:08b}'.format(read_bits(self.read(VERSION)))
        return int(value[:4], 2), int(value[4:], 2)

    # RegPaLevel 0x11

    def get_pa_0_on(self):
        return read_bits(self.read(PALEVEL), 0x00, 0x01)

    def get_pa_1_on(self):
        return read_bits(self.read(PALEVEL), 0x01, 0x01)

    def get_pa_2_on(self):
        return read_bits(self.read(PALEVEL), 0x02, 0x01)

    def get_output_power(self):
        return read_bits(self.read(PALEVEL), 0x03, 0x05)

    # RegLna 0x18

    def get_lna_current_gain(self):
        return read_bits(self.read(LNA), 0x02, 0x03)

    def get_lna_gain_select(self):
        return read_bits(self.read(LNA), 0x05, 0x03)

    # RegRxBw 0x19

    def get_dcc_freq(self):
        pass

    def set_dcc_freq(self, value):
        pass

    def get_rx_bw(self):
        pass

    def set_rx_bw(self, value):
        pass

    # RegAfcBw 0x1a

    def get_dcc_freq_afc(self):
        pass

    # RegDioMapping1 0x25

    def get_dio_mapping0(self):
        return read_bits(self.read(DIOMAPPING1), 0x00, 0x02)

    def set_dio_mapping0(self, value):
        self.set_bits(DIOMAPPING1, value, 0x00, 0x02)

    # RegIrgFlags1 0x27

    def get_mode_ready(self):
        return read_bits(self.read(IRQFLAGS1), 0x00, 0x01)

    def get_tx_ready(self):
        return read_bits(self.read(IRQFLAGS1), 0x02, 0x01)

    # RegIrqFlags2 0x28

    def get_fifo_full(self):
        return read_bits(self.read(IRQFLAGS2), 0x00, 0x01)

    def get_fifo_not_empty(self):
        return read_bits(self.read(IRQFLAGS2), 0x01, 0x01)

    def get_fifo_level(self):
        return read_bits(self.read(IRQFLAGS2), 0x02, 0x01)

    def get_fifo_overrun(self):
        return read_bits(self.read(IRQFLAGS2), 0x03, 0x01)

    def get_packet_sent(self):
        return read_bits(self.read(IRQFLAGS2), 0x04, 0x01)

    def get_payload_ready(self):
        return read_bits(self.read(IRQFLAGS2), 0x05, 0x01)

    def get_crc_ok(self):
        return read_bits(self.read(IRQFLAGS2), 0x06, 0x01)

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

    # RegFifoThresh 0x3c

    def get_tx_start_condition(self):
        return read_bits(self.read(FIFOTHRESH), 0x00, 0x01)

    def set_tx_start_condition(self, value):
        self.set_bits(FIFOTHRESH, value, 0x00, 0x01)

    def get_fifo_threshold(self):
        return read_bits(self.read(FIFOTHRESH), 0x01, 0x07)

    # RegTestPa1 0x5a

    def get_pa_20_dbm_1(self):
        return read_bits(self.read(TESTPA1))

    def set_pa_20_dbm_1(self, value):
        self.set_bits(TESTPA1, value)

    # RegTestPa1 0x5c

    def get_pa_20_dbm_2(self):
        return read_bits(self.read(TESTPA2))

    def set_pa_20_dbm_2(self, value):
        self.set_bits(TESTPA2, value)

    # RegTestDagc 0x6f

    def get_continuous_dagc(self):
        return read_bits(self.read(TESTDAGC))

    def set_continuous_dagc(self, value):
        self.set_bits(TESTDAGC, value)

    def send(self, data):
        tot_b = len(data)
        payload = bytearray([0x80, 0x00, tot_b]) + bytearray(data)
        self._spi_write(payload)
        self.set_mode('TX')
        while not self.get_mode_ready():
            utime.sleep(0.2)
            print('waiting mode ready')
        self.set_mode('TX')