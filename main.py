import sys
from machine import Pin, SPI, UART
import utime
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

def send(data):
    t.set_mode('STDBY')
    while not t.get_mode_ready():
        print('waiting stand by...')
        utime.sleep_ms(1)

    data_len = len(data)
    d = bytearray([data_len]) + bytearray(data)
    t.set_fifo(d)
    t.set_mode('TX')
    print(t.get_mode())
    while not t.get_mode_ready():
        print('waiting TX...')
        utime.sleep_ms(1)
    t.set_mode('STDBY')
    while not t.get_mode_ready():
        print('waiting stand by...')
        utime.sleep_ms(1)

# OPMODE 0x01

t.set_sequencer_off(0)
t.set_listen_on(0)
t.set_mode('STDBY')

# DATAMODUL 0x02

t.set_data_mode(0)
t.set_modulation_type('FSK')
t.set_modulation_shaping(1)

# BITRATEMSB 0x03, 0x04

t.set_bitrate(10000)

# FDEVMSB 0x05, 0x06

t.set_deviation(250000)

t.set_packet_length_conf('VARIABLE')

# SYNCCONFIG 0x2e
# SYNCVALUE 0x2f - 0x36

t.set_sync_size(2)
t.set_sync_value('0010110111010100')

# FIFOTHRESH 0x3c

t.set_tx_start_condition(1)
t.set_fifo_threshold(15)

# TESTPA1 0x5a

t.set_pa_20_dbm_1(0x55)

# TESTPA2 0x5c

t.set_pa_20_dbm_2(0x70)

# TESTDAGC 0x71

t.set_continuous_dagc(0x30)
