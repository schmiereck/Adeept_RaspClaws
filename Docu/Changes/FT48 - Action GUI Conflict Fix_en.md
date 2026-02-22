# FT48 - Action/GUI Servo Conflict Fix

**Feature**: Prevents robot "jittering" when Actions and GUI run in parallel  
**Date**: 2026-02-22  
**Status**: ✅ Implemented (Rev. 2 - Critical Bugfix)

## Problem

When GUIServer is running and ROS2 Actions execute movement commands simultaneously,
servo conflicts occur causing the robot to "jitter".

**Root Cause**: 
- Move thread (`RobotM`) runs continuously at 100 Hz (every 10ms)
- GUI commands can change servo positions at any time
- Actions use the same `commandInput()` functions
- Both compete for servo control
- No synchronization between GUI and Actions

## Solution (Rev. 2 - Fixed!)

**IMPORTANT**: Revision 2 fixes a critical bug in Rev. 1!

### Problem in Rev. 1
The check was performed in `Move.handle_movement_command()`.
This blocked **all** movement commands, including those from Actions themselves!
→ Actions couldn't move! ❌

### Solution in Rev. 2
The check was moved to **GUIServer**, where GUI commands arrive.
Actions call `move.commandInput()` directly, bypassing the GUI check.
→ Actions work, GUI is blocked! ✅

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

### 2. Ignore GUI Commands During Actions (in GUIServer!)

In `GUIServer.handle_movement_command()` (NOT in Move.py!):

```python
def handle_movement_command(data):
    """Handle movement commands from GUI"""
    
    # === CHECK IF ACTION IS RUNNING ===
    # Ignore GUI movement commands if a ROS2 Action is currently executing
    if move.is_action_in_progress():
        # Silently ignore to avoid log spam
        return True  # Pretend we handled it
    
    # ... rest of function processes GUI command
```

**Important**: The check is in **GUIServer**, not in Move.py!
Actions call `move.commandInput()` directly and bypass this check.

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
        # Actions call self.move.commandInput() directly
        # This bypasses the GUI check in GUIServer
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
   - ~~`handle_movement_command()`: Check for action lock~~ (Rev. 1 - REMOVED in Rev. 2)

2. **Server/GUIServer.py** (NEW in Rev. 2)
   - `handle_movement_command()`: Check for action lock **BEFORE** forwarding to Move

3. **Server/action_servers.py**
   - `execute_linear_move()`: try/finally with lock
   - `execute_rotate()`: try/finally with lock
   - `execute_arc_move()`: try/finally with lock
   - `execute_head_position()`: try/finally with lock

## Behavior

### Without Action (Normal)
```
GUI sends "forward" → GUIServer → Move thread responds → Robot moves ✓
```

### With Active Action
```
Action starts → Lock set
GUI sends "forward" → GUIServer checks lock → IGNORED (silently)
Action calls move.commandInput() directly → Move thread responds → Robot moves ✓
Action ends → Lock released
GUI sends "forward" → GUIServer → Move thread responds ✓
```

### On Action Cancel or Error
```
Action starts → Lock set
[Error or Cancel]
finally block → Lock released (guaranteed!)
GUI control restored ✓
```

## Important Change in Rev. 2

**Error in Rev. 1**: Check in `Move.handle_movement_command()`
- Blocked **all** commands (GUI + Actions)
- Actions couldn't move ❌

**Fix in Rev. 2**: Check in `GUIServer.handle_movement_command()`
- Blocks only **GUI commands**
- Actions work normally ✅

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
4. Simultaneously send GUI commands (or keep GUI running)

**Expected Result**: 
- Robot executes action smoothly ✓
- No more "jittering" ✓
- GUI commands ignored during action ✓
- Actions can move! ✓
- After action ends: GUI control fully functional again ✓

## Notes

- **Thread-safe**: Uses `threading.Lock()` for synchronization
- **Robust**: `finally` block guarantees lock release
- **Silent**: No log spam for ignored GUI commands
- **Transparent**: Clear log messages at action start/end
- **Correct**: Check in GUIServer, not in Move.py!

## Revision History

- **Rev. 1** (2026-02-22 09:03): Initial implementation - Check in Move.py (BUGGY)
- **Rev. 2** (2026-02-22 09:25): Bugfix - Check moved to GUIServer.py (CORRECT)

As of Rev. 2, the solution is simple, effective and works! ✅
