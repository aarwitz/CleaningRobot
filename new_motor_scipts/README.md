# Motor Control Scripts

Quick notes on what these scripts are for:

**nav2_compatible_velocity_controller.py** - The main one I'm actively working on. Takes Nav2's (v, omega) outputs and converts them to differential drive (v_left, v_right) in the [-100, 100] range that the motor driver expects.

**encoder_diagnostic_MOVING.py** / **encoder_diagnostic-READONLY.py** - I was debugging why the encoders were giving bad readings sometimes. Figured out that the encoder occasionally returns -1 values (most often from the back-right motor), but only when the robot is moving and you do a read right after a write. Quick reads after writes => bad data.

**average_velocity_control.py** - A simpler velocity controller that just averages/smooths commands on top of the base motor commands. Less sophisticated than the Nav2 version but easier to understand.

**PID_control.py** - PID-based velocity control attempt. does not work, i forget what i was doing.

**all_forward_then_backward.py** - Simple test script to verify basic motor movement. very similar to test_2.py
