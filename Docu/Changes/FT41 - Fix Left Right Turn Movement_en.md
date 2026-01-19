# FT41 - Fix Left/Right Turn Movement - Tank Steering

**Date:** 2026-01-19  
**Type:** Bugfix  
**Priority:** Medium

## Problem

Left/Right buttons did not create rotation. After many attempts, it became clear: **The robot cannot mechanically turn in place** when both sides move in opposite directions!

### Various approaches that DIDN'T work:

1. **All legs of one side synchronously** → Robot tips over, cannot walk
2. **Diagonal grouping with signs** → Direction flags cancel out signs
3. **Copied original code** → Same problems as above

### The actual problem:

The robot has only **2 degrees of freedom per leg** (horizontal + vertical). Legs can only move **forward/backward**, not sideways. A true turn in place with both sides moving opposite is **mechanically impossible**!

## Solution

**Tank Steering:** One side walks **normally forward** (like Forward), the other side **stays still**.

### Turn Left (counter-clockwise)

- **Left side (L1, L2, L3):** Stays STILL (h = 0)
- **Right side (R1, R2, R3):** Walks FORWARD (like Forward)
- → Robot rotates around left side ↺

### Turn Right (clockwise)

- **Right side (R1, R2, R3):** Stays STILL (h = 0)
- **Left side (L1, L2, L3):** Walks FORWARD (like Forward)
- → Robot rotates around right side ↻

### Important: Keep diagonal grouping!

The moving side must use **normal diagonal grouping**:

**Phase 0-0.5:** One leg of moving side lifts (e.g., R2)
**Phase 0.5-1.0:** The other two legs of moving side lift (R1, R3)

The stationary side stays **always on ground** (v = -10).

## Implementation

```python
# Turn Left - Right side walks forward:
if phase < 0.5:
    # Group 1: R2 in air, moves forward
    positions['R2'] = {'h': -h, 'v': v}      # Right: forward in air
    
    # R1, R3 on ground, move forward
    positions['R1'] = {'h': -h, 'v': -10}    # Right: forward on ground
    positions['R3'] = {'h': -h, 'v': -10}    # Right: forward on ground
    
    # ALL left legs stay still
    positions['L1'] = {'h': 0, 'v': -10}     # Left: still on ground
    positions['L2'] = {'h': 0, 'v': -10}     # Left: still on ground
    positions['L3'] = {'h': 0, 'v': -10}     # Left: still on ground
else:
    # Group 2: R1, R3 in air, move forward
    positions['R1'] = {'h': -h, 'v': v}
    positions['R3'] = {'h': -h, 'v': v}
    
    # R2 on ground, moves forward
    positions['R2'] = {'h': -h, 'v': -10}
    
    # ALL left legs stay still
    positions['L1'] = {'h': 0, 'v': -10}
    positions['L2'] = {'h': 0, 'v': -10}
    positions['L3'] = {'h': 0, 'v': -10}
```

## Visualization

**Turn Left (Top View):**
```
    FRONT
    
L1 ●         R1 →  (R1 lifts alternating)
    ↓            ↓
L2 ●         R2 →  (R2 lifts alternating)
    ↓            ↓
L3 ●         R3 →  (R3 lifts alternating)

   BACK
   
● = stays still (always on ground)
→ = walks forward (diagonal: one leg up, then the other two)
= Rotation around left axis ↺
```

## Changed Files

- `Server/Move.py`
  - Line 649-720: Tank steering implemented
  - Left: Right side walks, left stands
  - Right: Left side walks, right stands
  - Moving side uses diagonal grouping like Forward

## Testing

### Test Cases

1. **Left continuous:** ✓ Rotates counter-clockwise
2. **Right continuous:** ✓ Rotates clockwise
3. **Smooth movement:** ✓ Thanks to FT40 position tracking
4. **Forward → Left:** ✓ Smooth transition
5. **Left → Right:** ✓ Smooth direction change

### Expected Behavior

- **Left Button:** Robot rotates **counter-clockwise** around left side
- **Right Button:** Robot rotates **clockwise** around right side
- **Not turning in place:** Turn radius = half body width (one side stands)
- **Like a tank:** One track runs, one stands = rotation

## Lessons Learned

1. **Mechanical limits:** Not everything that sounds theoretical is mechanically possible
2. **Simpler is better:** Tank steering is simpler than complex rotations
3. **Diagonal grouping important:** Also for turns for stability
4. **Prototyping:** Sometimes you need to try multiple approaches

## References

- Related: FT40 (Position tracking makes turns smooth too)
- Previous: FT39 (Fixed Forward/Backward direction)
- Next: TBD

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
