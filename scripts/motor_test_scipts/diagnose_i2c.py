#!/usr/bin/env python3
"""
I2C Bus Diagnostic Tool
Checks for bus contention, firmware state, and communication reliability
"""
import smbus
import time
import struct
import subprocess

BUS_ID = 7
ADDR = 0x34

def check_i2c_bus():
    """Check what devices are on the I2C bus"""
    print("=== I2C Bus Scan ===")
    try:
        result = subprocess.run(['i2cdetect', '-y', str(BUS_ID)], 
                              capture_output=True, text=True, timeout=5)
        print(result.stdout)
    except Exception as e:
        print(f"Could not scan bus: {e}")
        print("Try: sudo apt-get install i2c-tools")

def test_basic_communication():
    """Test if we can communicate at all"""
    print("\n=== Basic Communication Test ===")
    bus = smbus.SMBus(BUS_ID)
    
    for attempt in range(10):
        try:
            # Try a simple read
            data = bus.read_byte(ADDR)
            print(f"Attempt {attempt+1}: SUCCESS - Read byte: 0x{data:02X}")
            time.sleep(0.5)
        except Exception as e:
            print(f"Attempt {attempt+1}: FAILED - {e}")
            time.sleep(0.5)

def test_encoder_stability():
    """Read encoders repeatedly without any motor commands"""
    print("\n=== Encoder Stability Test (No Motor Commands) ===")
    bus = smbus.SMBus(BUS_ID)
    
    failures = 0
    corruptions = 0
    
    for i in range(20):
        try:
            raw = bus.read_i2c_block_data(ADDR, 0x3C, 16)
            vals = struct.unpack('<iiii', bytes(raw))
            
            # Check for corruption
            if any(v == -1 for v in vals) or any(abs(v) > 10000000 for v in vals):
                print(f"Read {i+1}: CORRUPTED - {vals}")
                corruptions += 1
            else:
                print(f"Read {i+1}: OK - {vals}")
                
        except Exception as e:
            print(f"Read {i+1}: FAILED - {e}")
            failures += 1
            
        time.sleep(0.2)
    
    print(f"\nResults: {failures} failures, {corruptions} corruptions out of 20 attempts")

def test_motor_commands_only():
    """Send motor commands without reading encoders"""
    print("\n=== Motor Command Test (No Encoder Reads) ===")
    bus = smbus.SMBus(BUS_ID)
    
    # Initialize
    try:
        bus.write_byte_data(ADDR, 0x14, 1)
        bus.write_byte_data(ADDR, 0x15, 0)
        time.sleep(1.0)
        print("Initialized driver")
    except Exception as e:
        print(f"Init failed: {e}")
        return
    
    # Send commands
    for speed in [0, 20, 40, 60, 40, 20, 0]:
        try:
            bus.write_i2c_block_data(ADDR, 0x33, [speed, speed, speed, speed])
            print(f"Set motors to {speed}: SUCCESS")
            time.sleep(1.0)
        except Exception as e:
            print(f"Set motors to {speed}: FAILED - {e}")
            time.sleep(1.0)
    
    # Stop
    try:
        bus.write_i2c_block_data(ADDR, 0x33, [0, 0, 0, 0])
        print("Stopped motors")
    except:
        pass

def test_alternating_read_write():
    """Test if reading/writing in sequence causes issues"""
    print("\n=== Alternating Read/Write Test ===")
    bus = smbus.SMBus(BUS_ID)
    
    # Initialize
    try:
        bus.write_byte_data(ADDR, 0x14, 1)
        bus.write_byte_data(ADDR, 0x15, 0)
        time.sleep(1.0)
    except Exception as e:
        print(f"Init failed: {e}")
        return
    
    for i in range(10):
        # Write
        try:
            bus.write_i2c_block_data(ADDR, 0x33, [30, 30, 30, 30])
            print(f"Cycle {i+1}: Write SUCCESS", end=" | ")
            time.sleep(0.1)
        except Exception as e:
            print(f"Cycle {i+1}: Write FAILED - {e}", end=" | ")
        
        # Read
        try:
            raw = bus.read_i2c_block_data(ADDR, 0x3C, 16)
            vals = struct.unpack('<iiii', bytes(raw))
            if any(abs(v) > 10000000 for v in vals):
                print(f"Read CORRUPTED - {vals}")
            else:
                print(f"Read OK")
        except Exception as e:
            print(f"Read FAILED - {e}")
        
        time.sleep(1.0)
    
    # Stop
    try:
        bus.write_i2c_block_data(ADDR, 0x33, [0, 0, 0, 0])
    except:
        pass

if __name__ == "__main__":
    print("I2C Motor Driver Diagnostic Tool\n")
    
    # Run tests
    check_i2c_bus()
    time.sleep(1)
    
    test_basic_communication()
    time.sleep(1)
    
    test_encoder_stability()
    time.sleep(2)
    
    test_motor_commands_only()
    time.sleep(2)
    
    test_alternating_read_write()
    
    print("\n=== Diagnostic Complete ===")
