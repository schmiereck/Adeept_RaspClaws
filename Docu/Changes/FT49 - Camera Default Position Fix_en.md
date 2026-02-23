# FT49 - Camera Default Position Fix

**Feature**: Camera Home Position respects CAMERA_UP_DOWN_DEFAULT changes  
**Date**: 2026-02-22  
**Status**: ✅ Implemented

## Problem

When changing `CAMERA_UP_DOWN_DEFAULT` in Move.py (e.g., from 240 to 200), 
the camera home position did not update after server restart.

**Root Cause**:
- `pwm12` and `pwm13` variables are set **once** at module import time:
  ```python
  pwm12 = CAMERA_LEFT_RIGHT_DEFAULT  # Set at import
  pwm13 = CAMERA_UP_DOWN_DEFAULT     # Set at import
  ```
- `init_all()` and `look_home()` used these **static** variables
- When constants changed, the old values persisted
- Server uses power-saving mode: servos stay "soft" at startup
- Camera servos only activated when GUI connects or sends commands
- By that time, the old `pwm13` value was already baked in

## Solution

Changed `init_all()` and `look_home()` to use the **constants directly** 
instead of the static `pwm12`/`pwm13` variables.

### Changes in init_all()

**Before**:
```python
pwm.channels[12].duty_cycle = _pulse_to_duty_cycle(pwm12)
pwm.channels[13].duty_cycle = _pulse_to_duty_cycle(pwm13)
```

**After**:
```python
# Use constants directly for camera servos
pwm.channels[12].duty_cycle = _pulse_to_duty_cycle(CAMERA_LEFT_RIGHT_DEFAULT)
pwm.channels[13].duty_cycle = _pulse_to_duty_cycle(CAMERA_UP_DOWN_DEFAULT)

# Update tracking variables
Left_Right_input = CAMERA_LEFT_RIGHT_DEFAULT
Up_Down_input = CAMERA_UP_DOWN_DEFAULT
```

### Changes in look_home()

**Before**:
```python
default_positions = [
    pwm0, pwm1, ..., pwm11,
    pwm12, pwm13, pwm14, pwm15  # Static values from import
]
```

**After**:
```python
default_positions = [
    pwm0, pwm1, ..., pwm11,
    CAMERA_LEFT_RIGHT_DEFAULT,  # Always current value
    CAMERA_UP_DOWN_DEFAULT,     # Always current value
    pwm14, pwm15
]
```

## Benefits

1. ✅ **Dynamic**: Always uses current constant values
2. ✅ **Predictable**: Changes to constants take effect immediately
3. ✅ **Maintainable**: No need to track static variables
4. ✅ **Consistent**: Same behavior in `init_all()` and `look_home()`

## Testing

1. Change `CAMERA_UP_DOWN_DEFAULT` in Move.py
2. Commit and push changes
3. On Raspberry Pi:
   ```bash
   git pull
   sudo systemctl restart gui_server.service
   ```
4. Connect GUI and press "Home" button
5. Camera should move to NEW default position ✓

## Modified Files

- **Server/Move.py**:
  - `init_all()`: Use constants directly for servos 12/13
  - `look_home()`: Use constants directly for servos 12/13

## Notes

- Leg servos (pwm0-pwm11) still use static variables (intentional)
- Only camera servos need dynamic behavior for easy adjustment
- Power-saving mode still works correctly
- Backward compatible with existing code

## Why Only Camera Servos?

Camera servos are frequently adjusted for different robot configurations:
- Different camera mounting angles
- Different servo brands/tolerances
- User preferences for default viewing angle

Leg servos rarely need adjustment and benefit from compile-time constants.
