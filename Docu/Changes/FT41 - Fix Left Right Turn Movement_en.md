# FT41 - Fix Left/Right Turn Movement

**Date:** 2026-01-19  
**Type:** Bugfix  
**Priority:** Medium

## Problem

Left/Right buttons did not perform meaningful movements. The robot should **turn in place**, but made chaotic movements instead.

### Root Cause

The turn logic in `calculate_target_positions()` used the **wrong grouping** of legs:

```python
# OLD (WRONG):
if command == 'left':
    # Phase 0-0.5: L1, R2, L3 in air
    # Phase 0.5-1.0: R1, L2, R3 in air
    # = Diagonal grouping like Forward/Backward
```

**Problem:** For **turning in place**, legs must be grouped by **side**, not diagonally!

**Concept of rotation:**
- **Left side** (L1, L2, L3) moves **backward** (pushes back)
- **Right side** (R1, R2, R3) moves **forward** (pulls forward)
- → Robot **rotates counter-clockwise** (left)

## Solution

**Group by side instead of diagonal:**

### Turn Left (counter-clockwise)

```python
if phase < 0.5:
    # Left legs (L1, L2, L3) in air, moving BACKWARD
    h_left = +speed → -speed  # From front to back
    v_left = up
    
    # Right legs (R1, R2, R3) on ground, moving FORWARD
    h_right = -speed → +speed  # From back to front
    v_right = down
else:
    # Right legs in air, FORWARD
    # Left legs on ground, BACKWARD
```

### Turn Right (clockwise)

```python
if phase < 0.5:
    # Right legs (R1, R2, R3) in air, moving BACKWARD
    h_right = +speed → -speed  # From front to back
    v_right = up
    
    # Left legs (L1, L2, L3) on ground, moving FORWARD
    h_left = -speed → +speed  # From back to front
    v_left = down
else:
    # Left legs in air, FORWARD
    # Right legs on ground, BACKWARD
```

## Implementation

### Modified Function

**`calculate_target_positions(phase, speed, command)`** in `Server/Move.py`

**Turn Left:**
```python
elif command == 'left':
    if phase < 0.5:
        # Left legs in air, moving backward
        h_left = int(abs(speed) * math.cos(t * math.pi))  # +speed → -speed
        v_left = int(3 * abs(speed) * math.sin(t * math.pi))
        
        # Right legs on ground, moving forward
        h_right = -h_left  # -speed → +speed
        v_right = -10
        
        # Apply to ALL left/right legs
        positions['L1'] = positions['L2'] = positions['L3'] = {'h': h_left, 'v': v_left}
        positions['R1'] = positions['R2'] = positions['R3'] = {'h': h_right, 'v': v_right}
    else:
        # Right legs in air, moving forward
        # Left legs on ground, moving backward
```

**Turn Right:** Same logic, but sides swapped

## Changed Files

- `Server/Move.py`
  - Line 649-720: Complete reimplementation of turn logic
  - Left: Left side backward, right side forward
  - Right: Right side backward, left side forward

## Visualization

### Turn Left (Top View):

```
    FRONT
    
L1 ← ← ← R1 →
    ↑  ↑
L2 ← ← ← R2 →
    ↑  ↑
L3 ← ← ← R3 →

   BACK
   
Left legs (L1-L3): ← backward
Right legs (R1-R3): → forward
= Counter-clockwise rotation ↺
```

### Turn Right (Top View):

```
    FRONT
    
L1 → → → R1 ←
    ↑  ↑
L2 → → → R2 ←
    ↑  ↑
L3 → → → R3 ←

   BACK
   
Left legs (L1-L3): → forward
Right legs (R1-R3): ← backward
= Clockwise rotation ↻
```

## Testing

### Test Cases

1. **Left continuous:** ✓ Rotates counter-clockwise in place
2. **Right continuous:** ✓ Rotates clockwise in place
3. **Left → Stop → Left:** ✓ Smooth continuation (thanks to position tracking)
4. **Forward → Left:** ✓ Smooth transition
5. **Left → Right:** ✓ Smooth direction change
6. **No jerking:** ✓ Position tracking prevents jumps

### Expected Behavior

- **Left Button:** Robot rotates **counter-clockwise** (left)
- **Right Button:** Robot rotates **clockwise** (right)
- **Smooth movement:** Thanks to FT40 position tracking
- **In place:** Robot doesn't move forward/backward, only rotation

## Technical Details

### Movement Phases

**Phase 0.0 - 0.5:**
- One side lifts legs and moves them
- Other side on ground, pushes

**Phase 0.5 - 1.0:**
- Sides swap
- Continuous rotation

### Horizontal Positions

```python
# Always use abs(speed) for consistent movement
h = int(abs(speed) * math.cos(t * math.pi))

# For backward: use directly (+speed → -speed)
h_backward = h

# For forward: negate (-speed → +speed)
h_forward = -h
```

## Lessons Learned

1. **Grouping matters:** Forward/Backward = diagonal, Turn = by side
2. **Understand concept:** Rotation = opposite movement of sides
3. **Position tracking helps:** FT40 makes turns smooth on direction change
4. **Consistency:** Always `abs(speed)` for amplitude, sign for direction

## References

- Related: FT40 (Position tracking makes turns smooth too)
- Previous: FT39 (Fixed Forward/Backward direction)
- Next: TBD
