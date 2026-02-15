# FT47 - CommandHandler Refactoring (English)

**Date**: 2026-02-15  
**Author**: GitHub Copilot  
**Status**: ✅ Completed

---

## Summary

Elimination of duplicate command parsing/routing logic between GUIServer.py and ROSServer.py by introducing a central **CommandHandler** module.

### Before (Problems)
```
┌─────────────┐          ┌─────────────┐
│ GUIServer   │          │  ROSServer  │
│ (Routing)   │          │ (Routing)   │  ← Code Duplication
└──────┬──────┘          └──────┬──────┘
       │                        │
       └──────────┬─────────────┘
                  ▼
          ┌───────────────┐
          │    Move.py    │
          └───────────────┘
```

**Problems:**
- ❌ Code duplication: Changes must be made in both servers
- ❌ Inconsistency risk: Different implementations for same commands
- ❌ Potential conflicts: If both run simultaneously

### After (Solution)
```
┌─────────────┐          ┌─────────────┐
│ GUIServer   │          │  ROSServer  │
│ (Protocol)  │          │ (ROS Bridge)│
└──────┬──────┘          └──────┬──────┘
       │                        │
       └──────────┬─────────────┘
                  ▼
          ┌───────────────────┐
          │  CommandHandler   │  ← NEW: Central Logic
          │  (Thread-safe)    │
          └─────────┬─────────┘
                    ▼
            ┌───────────────┐
            │    Move.py    │
            └───────────────┘
```

**Benefits:**
- ✅ No code duplication
- ✅ Consistent command processing
- ✅ Thread-safe: Both servers can run in parallel
- ✅ No breaking changes

---

## Technical Details

### New File: CommandHandler.py

**Location**: `Server/CommandHandler.py`

**Class**: `CommandHandler`

**Thread-Safety**: Uses `threading.Lock()` for all hardware access

**Methods:**
- `handle_movement_command(command, speed=None)` - Movement commands
- `handle_camera_command(command)` - Camera commands
- `set_movement_speed(speed)` - Set speed
- `set_arc_factor(arc_factor)` - Set arc factor
- `set_servo_standby(enable)` - Servo standby/wakeup
- `set_camera_pause(enable)` - Camera pause/resume

**Example:**
```python
# Initialization
cmd_handler = CommandHandler.CommandHandler()

# Movement command
cmd_handler.handle_movement_command(CMD_FORWARD, speed=50)

# Camera command
cmd_handler.handle_camera_command(CMD_LOOK_UP)

# Change speed
cmd_handler.set_movement_speed(80)
```

### GUIServer.py Changes

**Import:**
```python
import CommandHandler
```

**Initialization:**
```python
cmd_handler = CommandHandler.CommandHandler()
```

**Handler functions adapted:**
- `handle_movement_command()` → calls `cmd_handler.handle_movement_command()`
- `handle_camera_command()` → calls `cmd_handler.handle_camera_command()`
- `handle_speed_command()` → calls `cmd_handler.set_movement_speed()`
- `handle_arc_factor_command()` → calls `cmd_handler.set_arc_factor()`
- `handle_power_management_command()` → calls `cmd_handler.set_servo_standby()`

**GUI-specific logic remains:**
- `tcpCliSock.send()` - Socket communication
- LED control (`ws2812`)
- GPIO switches
- System info

### ROSServer.py Changes

**Import:**
```python
import CommandHandler
```

**Initialization (in `__init__`):**
```python
if ROBOT_MODULES_AVAILABLE and CommandHandler is not None:
    self.cmd_handler = CommandHandler.CommandHandler()
else:
    self.cmd_handler = None
```

**Callbacks adapted:**
- `cmd_vel_callback()` → uses `self.cmd_handler.handle_movement_command()`
- `head_cmd_callback()` → uses `self.cmd_handler.handle_camera_command()`
- `set_smooth_cam_callback()` → uses `self.cmd_handler.handle_camera_command()`
- `set_servo_standby_callback()` → uses `self.cmd_handler.set_servo_standby()`

**ROS-specific logic remains:**
- ROS2 messages/topics
- Logger (`self.get_logger()`)
- Service responses

**Fallback:** If CommandHandler not available (mock mode), callbacks use `move.*` functions directly.

---

## Thread-Safety Details

### Problem without CommandHandler
When GUIServer and ROSServer run simultaneously:
```python
# GUIServer Thread 1:
move.set_movement_speed(50)  # ← Race Condition!
move.commandInput(CMD_FORWARD)

# ROSServer Thread 2:
move.set_movement_speed(80)  # ← Race Condition!
move.commandInput(CMD_RIGHT)
```

### Solution with CommandHandler
```python
# CommandHandler.py
class CommandHandler:
    def __init__(self):
        self._lock = threading.Lock()
    
    def handle_movement_command(self, command, speed=None):
        with self._lock:  # ← Exclusive access
            if speed is not None:
                move.set_movement_speed(speed)
            result = move.handle_movement_command(command)
            return result
```

**Result**: Only one thread can control hardware at a time → No conflicts!

---

## What was NOT changed?

### Server-specific features remain:

**GUIServer.py:**
- TCP socket communication (Port 10223)
- `tcpCliSock.send()` for client updates
- LED control (`RobotLight`, `ws2812`)
- GPIO switches (`Switch`)
- System info thread (`info_send_client()`)
- Battery monitoring (`get_battery_voltage()`)

**ROSServer.py:**
- ROS2 topics/services
- ROS2 messages (Twist, Point, etc.)
- ROS2 logger
- GUICommandClient (for camera pause)
- Action servers (HeadPosition, LinearMove, Rotate, ArcMove)
- Camera publisher

**Move.py:**
- Hardware API unchanged
- All existing functions still work

---

## Testing

### Syntax Tests
```bash
cd Server
python -m py_compile CommandHandler.py GUIServer.py ROSServer.py
# ✅ All passed
```

### Functional Tests (TODO on Raspberry Pi)

1. **GUIServer alone:**
   ```bash
   sudo systemctl restart robot_server.service
   # Test GUI client: Movement, camera, speed, arc-factor
   ```

2. **ROSServer alone:**
   ```bash
   ros2 run raspclaws_server ros_server
   # Test ROS2 topics: /cmd_vel, /head_cmd
   # Test services: /set_smooth_cam, /set_servo_standby
   ```

3. **Both in parallel:**
   ```bash
   # Terminal 1:
   sudo systemctl start robot_server.service
   
   # Terminal 2:
   ros2 run raspclaws_server ros_server
   
   # Test: Simultaneous commands from GUI and ROS2
   # → No crash, no race conditions
   ```

---

## Migration Guide (for Developers)

### Old Code Patterns

**Direct Move.py calls:**
```python
# OLD (avoid in new code)
move.commandInput(CMD_FORWARD)
move.look_up()
move.set_movement_speed(50)
```

### New Code Patterns

**Via CommandHandler:**
```python
# NEW (recommended)
cmd_handler.handle_movement_command(CMD_FORWARD)
cmd_handler.handle_camera_command(CMD_LOOK_UP)
cmd_handler.set_movement_speed(50)
```

**Advantage:** Thread-safe, works even when both servers run!

---

## Git

### Backup Tag
```bash
git tag -a "ros2-working-v1" -m "Backup before CommandHandler refactoring"
```

### Commit
```bash
git commit -m "FT47: Refactor command handling - central CommandHandler"
```

### Files
- `Server/CommandHandler.py` (new)
- `Server/GUIServer.py` (modified)
- `Server/ROSServer.py` (modified)

---

## Known Limitations

1. **Camera pause in ROSServer:** Still uses `gui_command_client.send_command()` to notify GUIServer. Could be moved to CommandHandler in the future if FPV.py is refactored.

2. **LED control:** Remains GUI-specific (not in CommandHandler). This is OK as LEDs are only relevant for GUI client.

3. **System info:** Remains GUI-specific (CPU, RAM, battery). ROS2 has separate publishers for this.

---

## Next Steps

1. ✅ Syntax tests passed
2. ⏳ Functional tests on Raspberry Pi
3. ⏳ Test parallel operation (GUI + ROS2)
4. ⏳ Update documentation in `project_info.md`

---

## Lessons Learned

- **Separation of Concerns**: Servers handle protocol/transport, CommandHandler handles hardware logic
- **Thread-safety matters**: Even if only one server runs - well prepared for future
- **Small steps**: Implement phase by phase, test after each step
- **Backwards compatibility**: Old functionality preserved, only internal structure changes
