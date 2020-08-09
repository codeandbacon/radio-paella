from micropython import const

FIFO = const(0x00)
OPMODE = const(0x01)
DATAMODUL = const(0x02)
BITRATEMSB = const(0x03)
BITRATELSB = const(0x04)

RFRMSB = const(0x07)
RFRMID = const(0x08)
RFRLSB = const(0x09)

IRQFLAGS1 = const(0x27)
IRQFLAGS2 = const(0x28)

PREAMBLEMSB = const(0x2c)
PREAMBLELSB = const(0x2d)
SYNCCONFIG = const(0x2e)
SYNCVALUE1 = const(0x2f)
SYNCVALUE2 = const(0x30)
SYNCVALUE3 = const(0x31)
SYNCVALUE4 = const(0x32)
SYNCVALUE5 = const(0x33)
SYNCVALUE6 = const(0x34)
SYNCVALUE7 = const(0x35)
SYNCVALUE8 = const(0x36)
PACKETCONFIG1 = const(0x37)