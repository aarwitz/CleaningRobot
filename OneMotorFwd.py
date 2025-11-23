#!/usr/bin/python3
# coding=utf8

# So this program reliably spins motor 1 forwards and no other motors move!
# Change the speed from 50 to -50 to move the motor backwards!!!
# Only script so far that does the same thing each time. Still very confused...

# Simple program to spin a single motor and stop it
# Adapted for Jetson Nano with Hiwonder Black Mecanum-Wheel Chassis

import sys
import time
import smbus2

ENCODER_MOTOR_MODULE_ADDRESS = 0x34

class EncoderMotorController:
    # HiWonder i2c port was 1, but ours is 7?
    def __init__(self, i2c_port=7, motor_type=3):
        self.i2c_port = i2c_port
        # Initialize the motor module
        with smbus2.SMBus(self.i2c_port) as bus:
            print(f"bus.write_i2c_block_data({ENCODER_MOTOR_MODULE_ADDRESS}, 20, [{motor_type}, ])")
            bus.write_i2c_block_data(ENCODER_MOTOR_MODULE_ADDRESS, 20, [motor_type, ])
    
    def set_speed(self, motor_id, speed):
        """
        Set speed for a single motor
        :param motor_id: Motor ID (1-4)
        :param speed: Speed value (-100 to 100, negative for reverse)
        """
        with smbus2.SMBus(self.i2c_port) as bus:
            try:
                if 0 < motor_id <= 4:
                    bus.write_i2c_block_data(ENCODER_MOTOR_MODULE_ADDRESS, 50 + motor_id, [speed, ])
                else:
                    raise ValueError("Invalid motor id. Must be 1-4")
            except Exception as e:
                print(f"Error setting motor speed: {e}")

if __name__ == '__main__':
    print("Initializing motor controller...")
    motor_controller = EncoderMotorController()
    
    # Select which motor to test (1-4)
    MOTOR_ID = 1
    SPEED = 50  # Speed value (adjust as needed, range typically -100 to 100)
    
    try:
        print(f"Spinning motor {MOTOR_ID} at speed {SPEED}...")
        motor_controller.set_speed(MOTOR_ID, SPEED)
        
        # Run motor for 2 seconds
        time.sleep(2)
        
        print(f"Stopping motor {MOTOR_ID}...")
        motor_controller.set_speed(MOTOR_ID, 0)
        
        print("Done!")
        
    except KeyboardInterrupt:
        print("\nInterrupted! Stopping motor...")
        motor_controller.set_speed(MOTOR_ID, 0)
    except Exception as e:
        print(f"Error: {e}")
        motor_controller.set_speed(MOTOR_ID, 0)

