#!/usr/bin/env python3
"""
Simple test: Spin ONE motor at a time, then stop it.
Based EXACTLY on HiWonder documentation section 4.5.4
"""
import smbus2
import time

"""
Still figuring this out:
The key insight: Don't call the "all motors" method (set_speed([0,0,0,0]) to register 51) before using individual motor commands (register 51, 52, 53, 54). It seems the motor controller has two different modes or the commands interfere with each other. Stick to only using individual motor commands (register 50 + motor_id) for individual control!
"""

# Constants from HiWonder documentation
ENCODER_MOTOR_MODULE_ADDRESS = 0x34
I2C_BUS = 7  # Your device is on bus 7, not 1
MOTOR_TYPE_JGB37 = 3

class EncoderMotorController:
    """Exact implementation from HiWonder docs"""
    def __init__(self, i2c_port, motor_type=3):
        self.i2c_port = i2c_port
        with smbus2.SMBus(self.i2c_port) as bus:
            bus.write_i2c_block_data(ENCODER_MOTOR_MODULE_ADDRESS, 20, [motor_type, ])
    
    def set_speed(self, motor_id, speed):
        """
        Simplified: ONLY individual motor control (like ClaudeAttemptOneMotor.py)
        motor_id: 1-4
        speed: speed value
        """
        with smbus2.SMBus(self.i2c_port) as bus:
            try:
                if 0 < motor_id <= 4:
                    bus.write_i2c_block_data(ENCODER_MOTOR_MODULE_ADDRESS, 50 + motor_id, [speed, ])
                else:
                    raise ValueError("Invalid motor id")
            except Exception as e:
                print(e)


if __name__ == "__main__":
    # Create motor controller
    motor_controller = EncoderMotorController(I2C_BUS, MOTOR_TYPE_JGB37)
    
    print("Starting motor test - one at a time...")
    print(f"Using I2C bus {I2C_BUS}")
    print("=" * 60)
    
    test_speed = 40
    
    # Test each motor in sequence
    for motor_id in range(1, 5):
        print(f"\nMotor {motor_id}:")
        print(f"  Starting motor {motor_id} at speed {test_speed}...")
        motor_controller.set_speed(motor_id, test_speed)
        
        # Let it run for 2 seconds
        time.sleep(2.0)
        
        print(f"  Stopping motor {motor_id}...")
        motor_controller.set_speed(motor_id, 0)
        
        # Wait 1 second before next motor
        time.sleep(1.0)
    
    print("\n" + "=" * 60)
    print("All motors tested! Complete!")



