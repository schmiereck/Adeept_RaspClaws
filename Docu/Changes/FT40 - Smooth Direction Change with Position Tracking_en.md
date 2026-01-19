# FT40 - Smooth Direction Change with Position Tracking

**Date:** 2026-01-19  
**Type:** Feature Enhancement  
**Priority:** High

## Problem

When switching between Forward and Backward, there was a **jerk**, even though movement within one direction was smooth.

### Root Cause

The previous solution used a **global phase** (`_movement_phase`) that was reset on direction change:

```python
# OLD (FT38-39):
_movement_phase = 0.0  # Global phase

# On direction change:
if direction_changed:
    _movement_phase = 0.0  # Reset → legs jump to start position!
```

**Problem:** The phase was reset, but the **actual leg positions** were not considered. The legs jumped to the start position of the new cycle, causing a visible jerk.

**Example:**
- Forward at phase=0.7: Leg at position +20 (almost front)
- Start Backward: Phase to 0.0 → Leg should be at -35
- **Jump from +20 to -35!**

## Solution

**Position-based tracking instead of phase tracking:**

Store the **actual servo positions** of each leg and interpolate smoothly from current position to target position on direction change.

### New Data Structure

```python
# Global dictionary for actual leg positions
_leg_positions = {
    'L1': 0,  # Left front horizontal position
    'L2': 0,  # Left middle horizontal position
    'L3': 0,  # Left rear horizontal position
    'R1': 0,  # Right front horizontal position
    'R2': 0,  # Right middle horizontal position
    'R3': 0   # Right rear horizontal position
}
```

### Algorithm

```python
def move_smooth(speed, command, cycle_steps=30):
    # 1. Calculate target positions for current phase
    target_positions = calculate_target_positions(phase, speed, command)
    
    # 2. Interpolate from current to target position
    for leg in ['L1', 'L2', 'L3', 'R1', 'R2', 'R3']:
        current = _leg_positions[leg]
        target = target_positions[leg]['h']
        
        # Smooth interpolation
        if command_changed and step < 5:
            alpha = 0.3 + (step / 5) * 0.7  # Stronger interpolation at start
        else:
            alpha = 1.0  # Normal: direct to target position
        
        new_position = current + alpha * (target - current)
        _leg_positions[leg] = new_position
        
        # 3. Apply position to servo
        apply_leg_position(leg, new_position, vertical)
```

### Advantages

1. **Smooth direction changes:** No more jumps, legs interpolate from current position
2. **Smooth stop/start:** Legs stay at their position, no reset
3. **Flexible interpolation:** Stronger interpolation in first steps after change
4. **Independent of phase:** Position is the "source of truth", not the phase

## Implementation

### New Functions

**`calculate_target_positions(phase, speed, command)`**
- Calculates target positions for all 6 legs at given phase
- Supports forward/backward ('no'), left, right
- Returns dictionary: `{'L1': {'h': ..., 'v': ...}, ...}`

**`apply_leg_position(leg, horizontal, vertical)`**
- Applies position to a specific leg
- Calls corresponding `dove_Left_X()` / `dove_Right_X()` function

### Modified Functions

**`move_smooth(speed, command, cycle_steps=30)`**
- Now uses `_leg_positions` dictionary instead of `_movement_phase`
- Detects command and direction changes
- Interpolates smoothly from current → target
- Stronger interpolation (alpha = 0.3 → 1.0) in first 5 steps after change

### New Imports

```python
import math  # For cos/sin calculations in calculate_target_positions()
```

## Changed Files

- `Server/Move.py`
  - Line 7: Added `import math`
  - Line 518-530: New global variables (`_leg_positions`, `_last_command`, `_last_speed_sign`)
  - Line 532-583: New `move_smooth()` with position tracking
  - Line 586-701: New helper functions `calculate_target_positions()` and `apply_leg_position()`

## Testing

### Test Cases

1. **Forward continuous:** ✓ Smooth without jumps
2. **Backward continuous:** ✓ Smooth without jumps
3. **Forward → Stop → Forward:** ✓ Smooth continuation
4. **Forward → Backward:** ✓ **Smooth interpolation, no jerk!**
5. **Backward → Forward:** ✓ **Smooth interpolation, no jerk!**
6. **Forward → Left:** ✓ Smooth transition
7. **Multiple direction changes:** ✓ Always smooth

### Expected Behavior

- **On direction change:** Legs interpolate smoothly from their current position to new target position
- **First 5 steps:** Stronger interpolation (alpha 0.3 → 1.0) for smooth transition
- **After that:** Normal movement with direct target position (alpha = 1.0)
- **No jerking** or jumps anymore on direction change!

## Technical Details

### Interpolation Formula

```python
new_position = current + alpha * (target - current)
```

- `alpha = 0.3`: 30% movement towards target (very smooth)
- `alpha = 1.0`: 100% movement towards target (direct)
- `alpha = 0.3 → 1.0`: Gradual transition over 5 steps

### Position Calculation

Forward/Backward (command='no'):
```python
# Phase 0-0.5: Group 1 (L1, R2, L3) in the air
h1 = int(speed * cos(phase * 2 * π))  # With sign for direction
v1 = int(3 * abs(speed) * sin(phase * 2 * π))  # Always positive

# Group 2 (R1, L2, R3) on ground
h2 = -h1  # Opposite
v2 = -10  # On ground
```

## Lessons Learned

1. **Position > Phase:** Store actual states, not abstract phase values
2. **Smooth Transitions:** Interpolation is better than reset
3. **Source of Truth:** Servo positions are the actual state
4. **Gradual Changes:** Stronger interpolation at start helps with smooth transitions

## Next Steps

- [ ] Optional: Speed parameter for variable speeds
- [ ] Optional: Adaptive interpolation based on position difference
- [ ] Optional: Position tracking also for vertical axis (currently only horizontal)

## References

- Previous Feature: FT39 (Movement direction with abs() for vertical movement)
- Next Feature: TBD
- Related: FT38 (Prevent steady() during movement)
