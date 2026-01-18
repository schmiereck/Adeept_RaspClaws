# Servo Tester Tool - Documentation

## Overview

The Servo Tester Tool is a standalone GUI application for testing and calibrating individual servos of the RaspClaws robot.

**File**: `Server/ServoTester.py`

**Features**:
- ✅ Individual control of all 8 servo channels
- ✅ Real-time PWM value display
- ✅ Sliders for precise positioning
- ✅ Quick buttons (Reset, Min, Max)
- ✅ Mock mode for development without hardware
- ✅ SSH-capable (X11 forwarding)

---

## Installation & Launch

### 1. On Raspberry Pi (with hardware)

**Via SSH with X11 forwarding**:

```bash
# From Windows (with X-Server like VcXsrv or Xming)
ssh -X pi@192.168.2.126

# On the Pi
cd /home/pi/adeept_raspclaws/Server
python3 ServoTester.py
```

**Directly on Pi** (with monitor):

```bash
cd /home/pi/adeept_raspclaws/Server
python3 ServoTester.py
```

### 2. On Windows (Mock Mode)

For development and testing without hardware:

```powershell
cd C:\Users\SCMJ178\IdeaProjects\Adeept_RaspClaws\Server
python ServoTester.py
```

**Note**: In mock mode, PWM values are only simulated and printed to console.

---

## GUI Overview

### Layout

```
┌─────────────────────────────────────────┐
│          Servo Tester                   │
│         [HARDWARE MODE]                 │
│   Move sliders to test each servo       │
├─────────────────────────────────────────┤
│                                         │
│  Head Up/Down (Ch 0)            [300]   │
│  [================|==============]      │
│  Min: 100              Max: 600         │
│                                         │
│  Head Left/Right (Ch 1)         [300]   │
│  [================|==============]      │
│  Min: 100              Max: 600         │
│                                         │
│  Leg 1 (Ch 4)                   [300]   │
│  [================|==============]      │
│  Min: 100              Max: 600         │
│                                         │
│  ... (more servos) ...                  │
│                                         │
├─────────────────────────────────────────┤
│ [Reset All]  [All to Min]  [All to Max] │
│                              [Quit]     │
└─────────────────────────────────────────┘
```

### Components

#### 1. **Servo Sliders**
- **Name**: Servo designation (e.g. "Head Up/Down")
- **Channel**: PCA9685 channel number (Ch 0-9)
- **Current Value**: Green number on right (current PWM value)
- **Slider**: Horizontal, range 100-600
- **Min/Max**: Display of valid range

#### 2. **Control Buttons**

**Reset All to Default**:
- Sets all servos to position 300 (center)
- Color: Blue
- Recommended at start/end

**All to Min**:
- Sets all servos to minimum position (100)
- Color: Orange
- Caution: May reach mechanical limits!

**All to Max**:
- Sets all servos to maximum position (600)
- Color: Orange
- Caution: May reach mechanical limits!

**Quit**:
- Exits application
- Resets all servos to default first
- Color: Red

---

## Servo Channels

### Channel Mapping

| Channel | Servo Name       | Function                    |
|---------|------------------|-----------------------------|
| 0       | Head Up/Down     | Vertical head movement      |
| 1       | Head Left/Right  | Horizontal head rotation    |
| 4       | Leg 1            | Front-right leg             |
| 5       | Leg 2            | Back-right leg              |
| 6       | Leg 3            | Back-left leg               |
| 7       | Leg 4            | Front-left leg              |
| 8       | Arm 1            | Right arm                   |
| 9       | Arm 2            | Left arm                    |

### PWM Values

- **Minimum**: 100 (one end position)
- **Maximum**: 600 (other end position)
- **Default**: 300 (center position)
- **Frequency**: 50 Hz (standard for servos)

**Important**: Actual mechanical limits may vary between servos. Test carefully!

---

## Usage Scenarios

### 1. Servo Test After Assembly

After assembly or replacing a servo:

1. ✅ Start ServoTester
2. ✅ Click "Reset All to Default"
3. ✅ Move each slider slowly from Min to Max
4. ✅ Check if servo responds and moves in correct direction
5. ✅ Note mechanical limits (if before 100 or 600)

### 2. Calibration

Find optimal PWM values for neutral positions:

1. ✅ Move slider to desired position
2. ✅ Note PWM value from green display
3. ✅ Transfer values to `Move.py` or `RPIservo.py`

### 3. Range of Motion Test

Test maximum movement ranges:

1. ✅ Set servo to Min (100)
2. ✅ Check if mechanical limit reached (sound/resistance)
3. ✅ If yes: Increase minimum in steps of 10
4. ✅ Repeat for maximum
5. ✅ Update constants in code

### 4. Troubleshooting

When having problems with a servo:

1. ✅ Test servo individually with ServoTester
2. ✅ No movement → Hardware problem (wiring/servo defective)
3. ✅ Jittering/noise → PWM problem or power supply
4. ✅ Wrong direction → Mechanically mounted incorrectly

---

## SSH X11 Forwarding Setup

### Windows with VcXsrv

1. **Install VcXsrv**:
   - Download: https://sourceforge.net/projects/vcxsrv/
   - Install with default settings

2. **Launch XLaunch**:
   - Multiple windows
   - Display number: 0
   - Start no client
   - ✅ Enable clipboard
   - ✅ Enable disable access control

3. **SSH Connection with X11**:
   ```powershell
   ssh -X pi@192.168.2.126
   ```

4. **Start ServoTester**:
   ```bash
   cd /home/pi/adeept_raspclaws/Server
   python3 ServoTester.py
   ```

### X11 Troubleshooting

**"Cannot connect to X server"**:
```bash
# Check DISPLAY variable
echo $DISPLAY
# Should be: localhost:10.0 or similar

# Set manually if needed
export DISPLAY=localhost:10.0
```

**"X11 forwarding request failed"**:
```bash
# Enable X11 forwarding on Pi
sudo nano /etc/ssh/sshd_config
# Change line: X11Forwarding yes
sudo systemctl restart ssh
```

---

## Code Architecture

### Class Structure

```python
# ServoPWM
- Wrapper for PCA9685
- Mock mode support
- set_pwm(channel, on, off)
- get_current_value(channel)

# ServoTesterGUI
- Main application
- create_widgets()           # GUI setup
- create_servo_control()     # Individual servo control
- on_slider_change()         # Slider event handler
- reset_all_servos()         # Reset function
- set_all_min/max()          # Min/Max functions
```

### Configuration

Adjustable constants at file beginning:

```python
# Add/change servo channels
SERVO_CHANNELS = {
    'New Servo': 10,  # New channel
}

# Adjust PWM ranges
PWM_MIN = 100
PWM_MAX = 600
PWM_DEFAULT = 300

# Change hardware address
PCA9685_ADDRESS = 0x40
```

---

## Extension Possibilities

### Planned Features (Optional)

- [ ] **Preset Positions**: Save/load servo configurations
- [ ] **Sequence Recorder**: Record and playback movement sequences
- [ ] **Individual Min/Max**: Different ranges per servo
- [ ] **Speed Control**: Adjust servo movement speed
- [ ] **Keyboard Shortcuts**: Quick control via keyboard
- [ ] **Config File**: External configuration file (JSON/YAML)

### Implementation Examples

**Save preset positions**:
```python
import json

def save_preset(name):
    preset = {ch: slider.get() for ch, slider in self.sliders.items()}
    with open(f'presets/{name}.json', 'w') as f:
        json.dump(preset, f)

def load_preset(name):
    with open(f'presets/{name}.json', 'r') as f:
        preset = json.load(f)
    for channel, value in preset.items():
        self.sliders[int(channel)].set(value)
```

---

## Safety Notes

### ⚠️ Important Warnings

1. **Mechanical Limits**: 
   - Servos can exert force!
   - Stop IMMEDIATELY on resistance
   - Test Min/Max carefully

2. **Power Supply**:
   - Servos need external power supply
   - Not just via USB!
   - Battery must be charged

3. **Wiring**:
   - Check before testing: All servos correctly connected?
   - PCA9685 correctly connected (SDA/SCL)?
   - No loose cables

4. **First Operation**:
   - Always start with "Reset All"
   - Small steps with new servos
   - Robot on safe surface

---

## Troubleshooting

### Problem: "Could not initialize PCA9685"

**Causes**:
- PCA9685 not connected
- Wrong I²C address
- I²C not enabled

**Solution**:
```bash
# Enable I²C
sudo raspi-config
# → Interface Options → I2C → Enable

# Scan I²C devices
sudo i2cdetect -y 1
# Should show 0x40

# If different address, adjust in code:
PCA9685_ADDRESS = 0x5F  # Or whatever is displayed
```

### Problem: GUI doesn't start via SSH

**Causes**:
- X11 forwarding not enabled
- DISPLAY variable missing
- Firewall blocking X11

**Solution**:
```bash
# Test X11 forwarding
ssh -X pi@192.168.2.126
echo $DISPLAY  # Should output something

# Simple X11 test
xclock  # Should display a clock

# If error, see "SSH X11 Forwarding Setup"
```

### Problem: Servo doesn't move

**Checklist**:
- [ ] PCA9685 correctly initialized? (not MOCK MODE)
- [ ] Servo connected to correct channel?
- [ ] External power supply connected?
- [ ] Battery charged?
- [ ] Servo defective? (test with different channel)

---

## Integration with Main Project

### Using in Move.py

Transfer optimal PWM values from ServoTester:

```python
# In Move.py or RPIservo.py
HEAD_VERTICAL_MIN = 150    # Determined from ServoTester
HEAD_VERTICAL_MAX = 550
HEAD_VERTICAL_DEFAULT = 300
```

### Calibration Workflow

1. ✅ Open ServoTester
2. ✅ Find optimal values for each servo
3. ✅ Note values
4. ✅ Transfer to `Move.py`
5. ✅ Restart main application
6. ✅ Test movements

---

## Summary

**ServoTester.py** is an essential tool for:
- ✅ Hardware testing after assembly
- ✅ Servo calibration
- ✅ Troubleshooting
- ✅ Range of motion determination

**Advantages**:
- 🎯 Easy operation
- 🔧 No code changes needed
- 🖥️ SSH compatible
- 🛡️ Mock mode for development
- 📊 Real-time feedback

**Recommendation**: Use as first tool after hardware setup!

---

**Author**: GitHub Copilot  
**Date**: 2026-01-18  
**Version**: 1.0  
**Status**: ✅ Production Ready
