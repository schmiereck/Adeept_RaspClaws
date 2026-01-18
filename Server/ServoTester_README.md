# Servo Tester - Quick Start

## 🚀 Quick Start

### On Raspberry Pi (SSH with X11)

**Mit Monitor am Pi**:
```bash
cd /home/pi/adeept_raspclaws/Server
python3 ServoTester.py
```

**Ohne Monitor (SSH + X11-Forwarding)**:
```bash
# Von Windows aus:
ssh -X pi@192.168.2.126
cd /home/pi/adeept_raspclaws/Server
python3 ServoTester.py
```

💡 **Detaillierte SSH X11-Anleitung**: [SERVO_TESTER_SSH_X11_DE.md](../Docu/SERVO_TESTER_SSH_X11_DE.md)
- X-Server Installation (VcXsrv)
- Schritt-für-Schritt Setup
- Troubleshooting
- Komfort-Verbesserungen

### On Windows (Mock Mode)

```powershell
cd C:\Users\SCMJ178\IdeaProjects\Adeept_RaspClaws\Server
python ServoTester.py
```

---

## 📋 Servo Channels

| Ch | Name             | Function          |
|----|------------------|-------------------|
| 0  | Head Up/Down     | Vertical head     |
| 1  | Head Left/Right  | Horizontal head   |
| 4  | Leg 1            | Front-right leg   |
| 5  | Leg 2            | Back-right leg    |
| 6  | Leg 3            | Back-left leg     |
| 7  | Leg 4            | Front-left leg    |
| 8  | Arm 1            | Right arm         |
| 9  | Arm 2            | Left arm          |

---

## 🎮 Controls

- **Sliders**: Move to test servo (100-600 PWM)
- **Reset All**: Set all to center position (300)
- **All to Min/Max**: Quick test of limits
- **Quit**: Exit and reset servos

---

## ⚠️ Safety

- Always start with "Reset All"
- Test slowly - servos can be strong!
- Stop on resistance
- Check power supply

---

## 📖 Full Documentation

- German: [SERVO_TESTER_DE.md](../Docu/SERVO_TESTER_DE.md)
- English: [SERVO_TESTER_EN.md](../Docu/SERVO_TESTER_EN.md)

---

**Status**: ✅ Ready to use
