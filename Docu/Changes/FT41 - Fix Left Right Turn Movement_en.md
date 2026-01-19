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

**Tank Steering with synchronous lifting/lowering:**

One side walks **normally forward** (like Forward), the other side **lifts/lowers synchronously but stays at h=0**.

### Problems with initial attempts:

1. **Too small amplitude:** Moving side got smaller h values than Forward
2. **Blocking:** Stationary side had all 3 legs on ground → blocked the turn!

### The Solution:

**Stationary side must lift/lower synchronously:**
- When moving side has 2 legs on ground pushing → Opposite side must **LIFT these 2 legs** (otherwise they block)
- When moving side lifts 2 legs → Opposite side must **LOWER these 2 legs** (for stability)
- **BUT:** Horizontally the stationary side stays at h=0 (doesn't move forward/back)

### Turn Left (counter-clockwise)

**Phase 0-0.5:** Group 1 (L1, R2, L3) in air
```python
# Moving side (right):
R2: h = +35 → -35, v = 0 → 100  # Right: moves forward in air
R1: h = +35,        v = -10      # Right: pushes on ground
R3: h = +35,        v = -10      # Right: pushes on ground

# Stationary side (left) - LIFTS SYNCHRONOUSLY:
L1: h = 0, v = 0 → 100  # Left: lifts, but h=0!
L3: h = 0, v = 0 → 100  # Left: lifts, but h=0!
L2: h = 0, v = -10      # Left: on ground, h=0
```

**Phase 0.5-1.0:** Group 2 (R1, L2, R3) in air
```python
# Moving side (right):
R1: h = +35 → -35, v = 0 → 100  # Right: moves forward
R3: h = +35 → -35, v = 0 → 100  # Right: moves forward
R2: h = +35,        v = -10      # Right: pushes on ground

# Stationary side (left) - LIFTS SYNCHRONOUSLY:
L2: h = 0, v = 0 → 100  # Left: lifts, but h=0!
L1: h = 0, v = -10      # Left: on ground, h=0
L3: h = 0, v = -10      # Left: on ground, h=0
```

**Result:** 
- Right side pushes forward with **full amplitude**
- Left side doesn't block because legs **lift synchronously**
- → Robot rotates around left side! ↺

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

