import smbus
import time
import struct

bus = smbus.SMBus(7)
ADDR = 0x34

# Init (run once)
bus.write_byte_data(ADDR, 0x14, 1)  # Hall encoder motors
bus.write_byte_data(ADDR, 0x15, 0)  # polarity


def read_encoder(bus, ADDR, reg):
    raw = bus.read_i2c_block_data(ADDR, reg, 4)
    return struct.unpack('<i', bytes(raw))[0]


SPEED = 60  # fixed test speed (-100..100)

def fwd():
    bus.write_i2c_block_data(
        ADDR, 0x33,
        [ SPEED, SPEED, SPEED, SPEED ]
    )

def rev():
    bus.write_i2c_block_data(
        ADDR, 0x33,
        [ -SPEED, -SPEED, -SPEED, -SPEED ]
    )

def stop():
    bus.write_i2c_block_data(
        ADDR, 0x33,
        [ 0, 0, 0, 0 ]
    )

start1 = read_encoder(bus, ADDR, 0x3C)
start2 = read_encoder(bus, ADDR, 0x40)
start3 = read_encoder(bus, ADDR, 0x44)
start4 = read_encoder(bus, ADDR, 0x48)

fwd()
time.sleep(2)

stop()
time.sleep(2)

rev()
time.sleep(2)

stop()
time.sleep(2)

end1 = read_encoder(bus, ADDR, 0x3C)
end2 = read_encoder(bus, ADDR, 0x40)
end3 = read_encoder(bus, ADDR, 0x44)
end4 = read_encoder(bus, ADDR, 0x48)

enc1 = end1 - start1
enc2 = end2 - start2
enc3 = end3 - start3
enc4 = end4 - start4

print("Motor 1: Start={}, End={}, Diff={}".format(start1, end1, enc1))
print("Motor 2: Start={}, End={}, Diff={}".format(start2, end2, enc2))
print("Motor 3: Start={}, End={}, Diff={}".format(start3, end3, enc3))
print("Motor 4: Start={}, End={}, Diff={}".format(start4, end4, enc4))

