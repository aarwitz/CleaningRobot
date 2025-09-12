from smbus2 import SMBus
import struct, time

BUS, ADDR = 7, 0x34

# Register addresses (decimal)
ADC_BAT, MOTOR_TYPE, ENCODER_POLARITY = 0, 20, 21          # 0: u16 LE battery mV; 20: u8 motor type; 21: u8 encoder polarity
PWM, SPEED, ENC_TOT           = 31, 51, 60         # 31: 4×i8 open-loop PWM; 51: 4×i8 closed-loop speed; 60: 4×i32 LE enc totals

# Values written to config registers
JGB37, ENC_POL = 3, 1  # JGB37 profile id = 3; encoder polarity bit = 1 (flip to 0 if forward gives negative counts)

def u8(x): 
    # Convert signed int8 (-128..127) to the on-wire byte (0..255). Example: -50 -> 206 (two’s complement).
    return x & 0xFF

with SMBus(BUS) as b:
    # --- One-byte configuration writes (exactly 1 byte each) ---
    b.write_i2c_block_data(ADDR, MOTOR_TYPE, [u8(JGB37)])   # reg 20 <- 0x03
    time.sleep(0.005)                                       # small settle delay ~5 ms
    b.write_i2c_block_data(ADDR, ENCODER_POLARITY,  [u8(ENC_POL)])  # reg 21 <- 0x01
    time.sleep(0.005)

    # Speed command arrays (m1, m2, m3, m4); board interprets roughly -100..+100
   # fwd  = [50,-50,-50,50]   # forward pattern
    #rev  = [-50,50,50,-50]   # reverse pattern
    
    fwd = [100, 100, -100, -100]

    #rev = [-50, 50, -50, 50] 
    stp = [0,0,0,0]
 
    # Battery read: 2 bytes, little-endian unsigned 16-bit millivolts
    d = b.read_i2c_block_data(ADDR, ADC_BAT, 2)             # returns [lo, hi]
    print("V(mV) =", d[0] | (d[1] << 8))

    # Encoders read: 16 bytes, 4× little-endian signed 32-bit totals
    raw = b.read_i2c_block_data(ADDR, ENC_TOT, 16)
    print("Enc =", struct.unpack('<iiii', bytes(raw)))       # '<' = LE, 'i' = 32-bit signed

    # Command closed-loop speed (reg 51): forward 5 s, then reverse 5 s
    b.write_i2c_block_data(ADDR, SPEED, [u8(x) for x in fwd])
    time.sleep(5.0)
    #b.write_i2c_block_data(ADDR, SPEED, [u8(x) for x in rev])
    #time.sleep(5.0)
    b.write_i2c_block_data(ADDR, SPEED, [u8(x) for x in stp])
    time.sleep(5.0) 
