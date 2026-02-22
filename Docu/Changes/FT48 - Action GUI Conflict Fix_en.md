# FT48 - Action/GUI Servo Conflict Fix

**Feature**: Prevents robot "jittering" when Actions and GUI run in parallel  
**Date**: 2026-02-22  
**Status**: ✅ Implemented

## Problem

When GUIServer is running and ROS2 Actions execute movement commands simultaneously,
servo conflicts occur causing the robot to "jitter".

**Root Cause**: 
- Move thread (`RobotM`) runs continuously at 100 Hz (every 10ms)
- GUI commands can change servo positions at any time
- Actions use the same `commandInput()` functions
- Both compete for servo control
- No synchronization between GUI and Actions

## Solution

### 1. Action Lock Mechanism in Move.py

New global variables and functions for action control:

```python
# Action control lock
action_in_progress = False
action_lock = threading.Lock()

def set_action_in_progress(in_progress: bool):
    """Set action in progress flag to prevent GUI interference"""
    global action_in_progress
    with action_lock:
        action_in_progress = in_progress
        if in_progress:
            print("[Move] ⚠️  ROS2 Action active - GUI movement commands suspended")
        else:
            print("[Move] ✓ ROS2 Action completed - GUI movement commands restored")

def is_action_in_progress() -> bool:
    """Check if an action is currently executing"""
    with action_lock:
        return action_in_progress
```

### 2. Ignore GUI Commands During Actions

In `handle_movement_command()`:

```python
def handle_movement_command(command):
    """Handle movement commands (forward, backward, stand, left, right, no)"""
    
    # Ignore GUI movement commands if a ROS2 Action is currently executing
    if is_action_in_progress():
        # Silently ignore (no spam in logs)
        return True  # Pretend we handled it
    
    # ... rest of the function
```

### 3. Actions Set/Release the Lock

All 4 actions extended with try/finally block:

**LinearMove, Rotate, ArcMove, HeadPosition**:

```python
async def execute_XXX_action(self, goal_handle):
    # ... setup code ...
    
    # === SET ACTION LOCK ===
    self.move.set_action_in_progress(True)
    
    try:
        # ... action execution ...
        return result
    finally:
        # === ALWAYS RELEASE LOCK ===
        self.move.set_action_in_progress(False)
```

The `finally` block guarantees the lock is **always** released,
even on errors or cancel requests.

## Modified Files

1. **Server/Move.py**
   - New functions: `set_action_in_progress()`, `is_action_in_progress()`
   - Global variables: `action_in_progress`, `action_lock`
   - `handle_movement_command()`: Check for action lock

2. **Server/action_servers.py**
   - `execute_linear_move()`: try/finally with lock
   - `execute_rotate()`: try/finally with lock
   - `execute_arc_move()`: try/finally with lock
   - `execute_head_position()`: try/finally with lock

## Behavior

### Without Action (Normal)
```
GUI sends "forward" → Move thread responds → Robot moves ✓
```

### With Active Action
```
Action starts → Lock set
GUI sends "forward" → IGNORED (silently) 
Action runs → Robot moves smoothly
Action ends → Lock released
GUI sends "forward" → Move thread responds ✓
```

### On Action Cancel or Error
```
Action starts → Lock set
[Error or Cancel]
finally block → Lock released (guaranteed!)
GUI control restored ✓
```

## Log Output

**Action Start**:
```
[Move] ⚠️  ROS2 Action active - GUI movement commands suspended
```

**Action End**:
```
[Move] ✓ ROS2 Action completed - GUI movement commands restored
```

**GUI Commands During Action**: Silently ignored (no log spam)

## Testing

1. Start GUIServer (on raspclaws-1)
2. Start ROSServer (also on raspclaws-1)
3. From ubuntu1: Execute Action
4. Simultaneously send GUI commands

**Expected Result**: 
- Robot executes action smoothly
- No more "jittering"
- GUI commands ignored during action
- After action ends: GUI control fully functional again

## Notes

- **Thread-safe**: Uses `threading.Lock()` for synchronization
- **Robust**: `finally` block guarantees lock release
- **Silent**: No log spam for ignored GUI commands
- **Transparent**: Clear log messages at action start/end

## Future Improvements (Optional)

Possible future enhancements:

1. **Queue System**: Store GUI commands in queue instead of ignoring
2. **Priorities**: Different lock levels for different action types
3. **Timeout**: Auto-unlock after X seconds if action hangs
4. **Status API**: Query function for GUI to check if action is running

For now, the solution is simple and effective! ✅
