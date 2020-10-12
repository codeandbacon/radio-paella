import sys
from machine import Pin, SPI, UART
import utime
import uos
import uasyncio
from rfm69hcw.rfm69hcw import RFM69HCW

# wemos d1 mini pro
# blue_led = Pin(2, Pin.OUT)
 
# nodemcu 0.9
red_led = Pin(16, Pin.OUT)
blue_led = Pin(2, Pin.OUT)

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
    rst = Pin(27, Pin.OUT)
else:
    cs = Pin(15, Pin.OUT)
    gdo0 = Pin(4, Pin.IN)
    rst = Pin(5, Pin.OUT)
    # gdo2 = Pin(5, Pin.IN)

spi = init_spi()

t = RFM69HCW(spi, cs, rst=rst)

t.reset()

# def send(data):
#     t.set_mode('STDBY')
#     while not t.get_mode_ready():
#         print('waiting stand by...')
#         utime.sleep_ms(1)

#     data_len = len(data)
#     d = bytearray([data_len]) + bytearray(data)
#     t.set_fifo(d)
#     t.set_mode('TX')
#     print(t.get_mode())
#     while not t.get_mode_ready():
#         print('waiting TX...')
#         utime.sleep_ms(1)
#     t.set_mode('STDBY')
#     while not t.get_mode_ready():
#         print('waiting stand by...')
#         utime.sleep_ms(1)

def recv():
    current_mode = t.get_mode()
    if current_mode != 'RX':
        t.set_mode('RX')
        while not t.get_mode_ready():
            print('waiting stand by...')
            utime.sleep_ms(1)

    print(t.get_mode())

    while not t.get_payload_ready():
        print('no packet, waiting...')
        utime.sleep_ms(1000)

    l = t.get_fifo()
    data = []
    for i in range(l):
        data.append(t.get_fifo())
    return data

# OPMODE 0x01

t.set_sequencer_off(0)
t.set_listen_on(0)
t.set_mode('STDBY')

# DATAMODUL 0x02

t.set_data_mode(0)
t.set_modulation_type('FSK')
t.set_modulation_shaping(1)

# BITRATEMSB 0x03, 0x04

t.set_bitrate(250000)

# FDEVMSB 0x05, 0x06

t.set_deviation(250000)

t.set_packet_length_conf('VARIABLE')

# AFCCTRL 0x0b

t.set_afc_low_beta_on(0)

# 0x11

t.set_pa_0_on(0)
t.set_pa_1_on(1)
t.set_pa_2_on(0)
t.set_output_power(31)

# 0x12

t.set_pa_ramp(9)

# 0x13

t.set_ocp_on(1)
t.set_ocp_trim(0x0a)

# 0x18

t.set_lna_zin(0)

# 0x19

t.set_dcc_freq(7)
t.set_rx_bw_mant(0)
t.set_rx_bw_exp(0)

# 0x1a

t.set_dcc_freq_afc(7)
t.set_rx_bw_mant_afc(0)
t.set_rx_bw_exp_afc(0)

# 0x25

t.set_dio_0_mapping(0)

# 0x2c

t.set_preamble_size(4)

# 0x2e

t.set_sync_on(1)
t.set_sync_size(1)

# SYNCCONFIG 0x2e
# SYNCVALUE 0x2f - 0x36

t.set_sync_size(2)
t.set_sync_value('0010110111010100')

# 0x37

t.set_packet_length_conf('VARIABLE')
t.set_dc_free('NONE')

# FIFOTHRESH 0x3c

t.set_tx_start_condition(1)
t.set_fifo_threshold(15)

# TESTPA1 0x5a

# t.set_pa_20_dbm_1(0x55)

# TESTPA2 0x5c

# t.set_pa_20_dbm_2(0x70)

# TESTDAGC 0x71

# t.set_continuous_dagc(0x30)

def init():
    return t