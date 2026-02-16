# Motor Driver Firmware Bug Report

## Issue Summary
The motor driver firmware (I2C address 0x34) crashes after sustained high-speed motor operation, requiring hardware power cycle to recover.

## Hardware Details
- **I2C Bus**: 7
- **Device Address**: 0x34
- **Commands Used**: 
  - Init: 0x14 (value 1), 0x15 (value 0)
  - Motor control: 0x33 (4-byte block write)
  - Encoder read: 0x3C (16-byte block read)

## Reproduction Steps

### What Works (Stable)
✅ I2C communication at idle (100% success rate)  
✅ Reading encoders when motors stopped (100% success rate)  
✅ Motor commands at speeds ≤35 for short durations (<3 seconds)  
✅ Alternating read/write at low speed (≤30) with 1+ second intervals

### What Crashes Firmware (Reproducible)
❌ Sustained motor operation at speed ≥40 for >4-5 seconds  
❌ Continuous read/write operations during high-speed motor operation  
❌ Reverse direction (-40 or lower) seems to crash faster than forward

## Observed Behavior

### Before Crash
1. Encoder reads start returning corrupted values:
   - Examples: -62268, -6416669, -1895825024, 335544693
   - Values exceed reasonable encoder counts by orders of magnitude
   - Suggests memory corruption or buffer overflow in firmware

### During Crash
2. I2C communication fails with "Connection timed out" (errno 110)
3. All I2C operations fail, including simple writes
4. No recovery possible through I2C commands

### After Crash
5. **Hardware power cycle required** - software reset commands fail
6. After power cycle, device operates normally again

## Example Test Output

```
T=4.00s | Cmd=  20 | dL=-62268 dR=+3268 | OK        <- Corruption starts
T=5.00s | Cmd=  25 | -- READ_FAIL --
T=6.00s | Cmd=  30 | dL=+75120 dR=+9583 | OK        <- More corruption
T=7.01s | Cmd=  35 | dL=+6414 dR=+6413 | OK
T=8.01s | Cmd=  40 | dL=-6416669 dR=+7394 | OK      <- Severe corruption
T=9.01s | Cmd=  45 | dL=+6434076 dR=+10012 | OK
  Write attempt 1 failed: [Errno 110] Connection timed out  <- CRASH
```

## Diagnostic Results

Full diagnostic run (`diagnose_i2c.py`) shows:
- Bus scan: Only device 0x34 present (no contention)
- Basic communication: 10/10 success
- Encoder stability test: 20/20 success with no corruption
- Motor command test: 7/7 success
- Alternating read/write (low speed): 10/10 success

**Conclusion**: Hardware and I2C bus are functional. Issue is firmware bug under load.

## Root Cause Hypothesis

The firmware likely has one or more of these issues:
1. **Buffer overflow** when handling encoder updates at high motor speeds
2. **Interrupt handler bug** that corrupts memory during simultaneous I2C read and motor control
3. **Stack overflow** or memory leak during extended operation
4. **Race condition** between encoder sampling and I2C communication

## Workarounds Implemented

Until firmware is fixed, we must:

1. **Limit maximum speed** to ≤35 (avoid crash threshold at ~40)
2. **Add rest periods** every 3 seconds (stop motors, let firmware stabilize)
3. **Never read encoders during high-speed operation** (stop motors first)
4. **Avoid rapid read/write alternation** at high speeds

See `firmware_safe_control.py` for working implementation.

## Recommended Firmware Fixes

1. Implement watchdog timer to auto-reset on firmware hang
2. Fix memory corruption in encoder interrupt handler
3. Add overflow protection for encoder buffers
4. Test sustained operation at all speed levels
5. Improve I2C transaction handling to prevent race conditions

## Test Scripts Provided

- `diagnose_i2c.py` - Comprehensive I2C diagnostic tool
- `firmware_safe_control.py` - Safe operation within firmware limitations
- `working_v2_recovery.py` - Demonstrates crash with recovery attempts
- `working_v3_separated.py` - Separated read/write phases

## Contact Information

Please contact motor driver manufacturer with this report and request firmware update.

---

**Date**: February 16, 2026  
**Testing Environment**: Linux on I2C bus 7  
**Test Duration**: Multiple hours across different speed profiles
