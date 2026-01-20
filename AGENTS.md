# Agent Development Environment Documentation

## Project Overview

**Adeept_RaspClaws** - Robot control software consisting of:
- **Client**: Windows GUI (Tkinter) for remote control
- **Server**: Raspberry Pi server for robot control and video streaming

GitHub Repository: https://github.com/schmiereck/Adeept_RaspClaws.git

---

## 1. Development Environments

### 1.1 Windows Development Machine

**Operating System**: Windows (PowerShell v5.1)

**IDE**: IntelliJ IDEA 2025.3

**Python Version**: Python 3.7.9 (64-bit)

**Project Directory**:
```
C:\Users\SCMJ178\IdeaProjects\Adeept_RaspClaws\
```

**Virtual Environment**:
```
C:\Users\SCMJ178\IdeaProjects\Adeept_RaspClaws\.venv\
```

**Python SDK Path**:
```
C:\Users\SCMJ178\IdeaProjects\Adeept_RaspClaws\.venv\Scripts\python.exe
```

**Important Python Packages**:
```
opencv-python==4.12.0.88
numpy==1.21.6
pyzmq
tkinter (standard library)
```

**Installing Dependencies**:
```powershell
pip install -r requirements.txt
```

### 1.2 Raspberry Pi Server

**Operating System**: Debian GNU/Linux (Raspberry Pi OS)
- Kernel: 6.12.47+rpt-rpi-v8 #1 SMP PREEMPT Debian 1:6.12.47-1+rpt1

**Python Version**: Python 3.13

**Hostname**: raspberrypi

**Default User**: pi

**Project Directory on Pi**:
```
/home/pi/adeept_raspclaws/
```

**Old Project Directory (Backup)**:
```
/home/pi/adeept_raspclaws-original/
```

**Git Branch**: master

**Important Notes**:
- ⚠️ Directory names are now **capitalized**: `Server/` instead of `server/`
- ⚠️ Main file is now called `GUIServer.py` (formerly `server.py`)
- Backward compatibility link: `/home/pi/adeept_raspclaws/server/server.py` → `GUIServer.py`

**Hardware**:
- Camera: OV5647 (libcamera v0.6.0+rpt20251202)
- PCA9685 PWM Controller: Address **0x40** (formerly 0x5F)
- I²C Bus: busnum=1
- **ADS7830 ADC**: Address **0x48** (battery monitoring, 8-channel 8-bit)
- WS2812 LEDs: 16 LEDs (via SPI, optional)
- MPU6050: Gyroscope/Accelerometer (optional)

**I²C Device Check**:
```bash
sudo i2cdetect -y 1
```
Expected addresses:
- 0x40: PCA9685 PWM Controller
- 0x48: ADS7830 ADC (battery monitoring)
- 0x68: MPU6050 (optional)

---

## 2. Network Configuration

### 2.1 Standard Operation (Direct Connection)

**Raspberry Pi IP**: 192.168.2.126 (or variable)

**Client IP**: 192.168.2.107 (or variable)

**Router Gateway**: 192.168.2.1

**WLAN**: speedport.ip

**Ports**:
- TCP 10223: Main connection (Client ↔ Server commands)
- TCP 5555: Video stream (ZMQ PUB/SUB)
- Optional: TCP 8080 (Web interface)

**IP.txt Configuration** (Client):
```
192.168.2.126
```

### 2.2 SSH Tunnel (for machines with firewall restrictions)

**Problem**: Some development machines have firewall restrictions without admin rights.

**Solution**: SSH tunnel with port forwarding

**Tunnel Command**:
```powershell
ssh -L 10223:localhost:10223 -L 5555:localhost:5555 pi@192.168.2.126
```

**IP.txt Configuration** (for SSH tunnel):
```
127.0.0.1
```

**Automation**: `Client/start_ssh_tunnel.bat` (interactive, password required)

⚠️ **Important**: SSH tunnel must be started manually and runs interactively (password entry required). SSH keys can be configured for passwordless authentication (see `Docu/Changes/FT5 - SSH Keys Setup_en.md`).

**Error Message "channel 5: open failed"**:
- This message may occur when the server is not ready yet or the video stream has not been initialized
- Normal during connection setup, as long as the GUI connects afterwards

---

## 3. Server Operation on Raspberry Pi

### 3.1 Systemd Service

**Service Name**: `robot_server.service`

**Service Configuration**: `/etc/systemd/system/robot_server.service` (presumably)

**Commands**:
```bash
# Start service
sudo systemctl start robot_server.service

# Stop service
sudo systemctl stop robot_server.service

# Restart service
sudo systemctl restart robot_server.service

# Check status
sudo systemctl status robot_server.service

# Show logs
sudo journalctl -u robot_server.service -n 100 --no-pager

# Follow logs live
sudo journalctl -u robot_server.service -f
```

**Start Command** (manual):
```bash
sudo python3 /home/pi/adeept_raspclaws/server/server.py
```
(The link `server.py` points to `GUIServer.py`)

### 3.2 Project Update from Git

```bash
cd /home/pi/adeept_raspclaws
git pull
sudo systemctl restart robot_server.service
```

**Switch Branch**:
```bash
git branch  # Show current branch
git checkout master  # Switch to master
```

---

## 4. Client Operation (Windows GUI)

### 4.1 Starting the GUI

**In IntelliJ**:
- Run Configuration: `GUI.py`
- Debug Mode: Shows output
- Run Mode: Also works since IntelliJ restart

**In Terminal**:
```powershell
cd C:\Users\SCMJ178\IdeaProjects\Adeept_RaspClaws\Client
..\..venv\Scripts\python.exe GUI.py
```

### 4.2 IP Configuration

**File**: `Client/IP.txt`

For **direct connection**:
```
192.168.2.126
```

For **SSH tunnel**:
```
127.0.0.1
```

**Important**: The GUI reads the IP from this file at startup.

### 4.3 Video Stream (Footage-GUI)

The video stream is started as a **separate process**:
- Automatically started by `GUI.py`
- Uses `Footage-GUI.py`
- Displays camera feed in separate window

**Note**: The server sends `VIDEO_READY` signals multiple times (10x over 10 seconds) to ensure the client is ready.

---

## 5. Architecture and Communication

### 5.1 Communication Flow

```
┌─────────────────┐         TCP 10223          ┌─────────────────┐
│  Windows Client │ ◄────────────────────────► │  Raspberry Pi   │
│    (GUI.py)     │     Commands + Info        │ (GUIServer.py)  │
└─────────────────┘                            └─────────────────┘
         │                                              │
         │          ZMQ PUB/SUB (Port 5555)            │
         │          Video Frames (JPEG)                │
         ▼                                              ▼
┌─────────────────┐                            ┌─────────────────┐
│ Footage-GUI.py  │ ◄────────────────────────► │    FPV.py       │
│ (Video Window)  │                            │ (Video Capture) │
└─────────────────┘                            └─────────────────┘
```

### 5.2 Threading Model (Server)

**GUIServer.py**:
1. **Main Thread**: Socket connection and command processing
2. **FPV Thread**: Video capture and streaming (runs continuously, even without client)
3. **Info Thread**: Sends system information (CPU, RAM, Temp) to client

**Important**: FPV thread is started **once at server startup** and runs permanently. No new thread is started on client reconnect.

### 5.3 Reconnection Handling

**Problem Fixed**: Client can connect/disconnect multiple times without server restart.

**Solution**:
- FPV thread runs continuously (ZMQ PUB/SUB allows multiple subscribers)
- Socket is properly closed after disconnect
- 2 second wait before new accept
- LED status display (blue = connected, breathing = waiting)

---

## 6. Features and Functions

### 6.1 Movement Control

**Commands**:
- `forward`, `backward`: Forward/backward
- `left`, `right`: Turn left/right
- `DS`: Direction Stop (stand still)
- `TS`: Turn Stop (stop turning)

**Speed**:
- `fast`: Fast mode
- `slow`: Slow mode

**Smooth Mode**:
- `smooth`: Smooth acceleration/deceleration for movement
- `smoothOff`: Smooth mode off

Documentation: `Docu/Changes/FT20 - SmoothMode Analyse_de.md`

### 6.2 Camera Control

**Movement**:
- `up`, `down`: Camera up/down
- `lookleft`, `lookright`: Camera left/right
- `home`: Camera to neutral position

**Modes**:
- `steadyCamera`: Camera stabilization on
- `steadyCameraOff`: Camera stabilization off
- `smoothCam`: Smooth camera movements
- `smoothCamOff`: Smooth camera movements off

Documentation: `Docu/Changes/FT31 - SmoothCam Implementation_de.md`

### 6.3 Computer Vision Features

- `findColor`: Enable color detection
- `motionGet`: Motion detection (watchdog)
- `CVFL`: Find Line Mode (line following)
- `stopCV`: Stop all CV features

**Set Color**:
```python
command = {'data': [r, g, b]}  # RGB values
```

### 6.4 LED Control (WS2812)

- `police`: Police light effect
- `policeOff`: Return to normal breathing effect

**Status Colors**:
- Blue (64,128,255): Client connected
- Breathing (70,70,255): Waiting for client
- Green: Access point mode active

⚠️ **Note**: WS2812 LEDs are optional. Server runs without functioning LEDs in mock mode.

### 6.5 System Information

**Client receives every 1-3 seconds**:
```
INFO:<CPU_TEMP> <CPU_USAGE> <RAM_USAGE> <BATTERY_VOLTAGE>
```

Example: `INFO:58.0 4.3 42.6 7.8`
- CPU Temperature: 58.0°C
- CPU Usage: 4.3%
- RAM Usage: 42.6%
- Battery Voltage: 7.8V

**GUI Display**:
- CPU Temp, CPU Usage, RAM Usage: Always visible
- **Battery**: With color coding (Green/Orange/Red) based on charge level
  - Green: ≥ 60% (≥ 7.44V)
  - Orange: 30-60% (6.72V - 7.44V)
  - Red: < 30% (< 6.72V)
  - Gray: "N/A" when hardware not available

Documentation: `Docu/Changes/FT33 - Battery Monitor_de.md` / `Docu/Changes/FT33 - Battery Monitor_en.md`

---

## 7. Keyboard Shortcuts (GUI)

See documentation:
- German: `Docu/KEYBOARD_SHORTCUTS.md`
- English: `Docu/KEYBOARD_SHORTCUTS_EN.md`

**Most Important Shortcuts**:
- `W/A/S/D`: Movement
- `Arrow Keys`: Camera control
- `Q`: Exit program

---

## 8. Known Issues and Solutions

### 8.1 PCA9685 Address Changed

**Problem**: Hardware uses address 0x40, not 0x5F

**Solution**: Changed in `RPIservo.py` and `GUIServer.py`

Documentation: `Docu/Changes/FT6 - PCA9685 Fix_en.md`

### 8.2 Reconnection Problem (SOLVED)

**Problem**: Video stream did not work on second connect

**Solution**: FPV thread runs continuously, not per client

Documentation: `Docu/Changes/FT13 - Reconnection Fix_de.md`, `Docu/Changes/FT12 - FPV Thread Fix_de.md`

### 8.3 ZMQ Video Stream Binding

**Problem**: Video stream had to work for direct connection and SSH tunnel

**Solution**: Server binds to `tcp://*:5555` (all interfaces), client connects to IP from IP.txt

Documentation: `Docu/Changes/FT14 - ZMQ PubSub Fix_de.md`

### 8.4 CPU Load Optimization

**Problem**: Server had 99% CPU load

**Solution**: Strategic `time.sleep(0.01)` inserted in main loops

Documentation: `Docu/Changes/FT32 - CPU Optimization_de.md`

### 8.5 LED Problem

**Problem**: WS2812 LEDs did not work (SPI not available)

**Solution**: Mock mode implemented, server runs without LEDs

Documentation: `Docu/Changes/FT19 - LED Problem_en.md`

### 8.6 Connection Error Handling

**Problem**: Various socket errors on disconnect

**Solution**: Comprehensive exception handling for all socket error types

Documentation: `Docu/Changes/FT15 - Connection Error Handling_de.md`

---

## 9. Change History

All feature changes are documented under `Docu/Changes/`:

**Important Features** (selection):
1. **FT1-FT4**: Connection and Video Stream Fixes
2. **FT5**: SSH Keys Setup
3. **FT6**: PCA9685 Fix (I2C address 0x40)
4. **FT12-FT15**: Reconnection, FPV Thread, ZMQ PubSub Fixes
5. **FT16-FT18**: Servo Tester Tool
6. **FT19**: LED Problem Fix
7. **FT20**: SmoothMode Analysis
8. **FT21-FT30**: Smooth Movement Implementation and Optimizations
9. **FT31**: SmoothCam Implementation
10. **FT32**: CPU Optimization
11. **FT33**: Battery Monitor
12. **FT34-FT46**: Additional bugfixes, refactorings and improvements

Many changes exist in German (_de) and English (_en).

Complete list: See `Docu/Changes/` directory.

---

## 10. Development Workflow

### 10.1 Typical Workflow

1. **Develop changes on Windows**:
   ```powershell
   cd C:\Users\SCMJ178\IdeaProjects\Adeept_RaspClaws
   # Make changes, test with local GUI
   ```

2. **Commit and Push**:
   ```powershell
   git add .
   git commit -m "Description"
   git push origin master
   ```

3. **Update on Raspberry Pi**:
   ```bash
   ssh pi@192.168.2.126
   cd /home/pi/adeept_raspclaws
   git pull
   sudo systemctl restart robot_server.service
   ```

4. **Check logs**:
   ```bash
   sudo journalctl -u robot_server.service -f
   ```

### 10.2 Testing

**Without Hardware** (Mock Mode):
- Server runs even without PCA9685, WS2812, camera
- Ideal for logic tests

**With Hardware**:
- Complete test of all features
- Verify camera feed
- Test servo movements

### 10.3 Debug Tips

**Client Side**:
- Debug mode in IntelliJ shows all output
- `print()` statements are displayed in run window

**Server Side**:
- `journalctl` for systemd service logs
- Use `sys.stdout.flush()` after important prints
- Errors are saved as systemd logs

**Network Tests**:
```powershell
# Test if port is open
Test-NetConnection -ComputerName 192.168.2.126 -Port 10223

# Ping test
ping 192.168.2.126

# Check port listening (Windows)
netstat -ano | findstr :10223
```

---

## 11. Important Files

### 11.1 Configuration Files

- `Client/IP.txt`: Server IP address
- `requirements.txt`: Python dependencies
- `setup.py`: Package setup (if present)

### 11.2 Main Programs

**Client**:
- `Client/GUI.py`: Main program with Tkinter GUI
- `Client/Footage-GUI.py`: Video window (separate process)
- `Client/ip_utils.py`: IP configuration utilities

**Server**:
- `Server/GUIServer.py`: Main server (formerly server.py)
- `Server/FPV.py`: Video capture and streaming
- `Server/Move.py`: Movement control
- `Server/RPIservo.py`: Servo controller (PCA9685)
- `Server/RobotLight.py`: LED control (WS2812)
- `Server/Switch.py`: GPIO switches
- `Server/Voltage.py`: Battery monitoring
- **`Server/ServoTester.py`**: Servo test tool with GUI (standalone)

**Tools**:
- `Server/ServoTester.py`: Standalone GUI for testing individual servos
  - Sliders for all 8 servo channels
  - SSH-capable (X11 forwarding)
  - Mock mode for development without hardware
  - Documentation: `Docu/Changes/FT16 - Servo Tester_de.md` / `Docu/Changes/FT16 - Servo Tester_en.md`

### 11.3 Documentation

All documents under `Docu/`:
- English versions end with `_en.md`
- German versions without suffix or with `_de.md`
- Feature history under `Docu/Changes/`
  - For each new feature, a "FT<nr> - <description>_<language>.md" file is created, which is updated when the feature changes.

For a new feature, create a new FT file.
Note: You don't need to document every bugfix in the FT files. They should describe features and their functionality.

---

## 12. Future Development

### 12.1 ROS 2 Integration (Planned)

**Preparation**:
- Server architecture is already modular
- Move.py can be adapted as ROS2 node
- FPV stream can be published as ROS2 topic

**Proposal**:
- Create separate ROS2 bridge node
- Keep existing socket communication (for legacy clients)
- Add parallel ROS2 topics

### 12.2 Improvement Ideas

- [ ] Web-based GUI (Flask/FastAPI)
- [ ] Gamepad/Controller support
- [ ] Autonomous navigation modes
- [ ] Improved computer vision (object detection)
- [ ] Video recording function
- [ ] Telemetry logging
- [ ] Multi-client support

---

## 13. Troubleshooting Checklist

### Client Won't Start

- [ ] Virtual environment activated?
- [ ] OpenCV installed? (`pip install opencv-python`)
- [ ] ZMQ installed? (`pip install pyzmq`)
- [ ] IP.txt correctly configured?

### No Video Stream

- [ ] Server running? (`sudo systemctl status robot_server.service`)
- [ ] SSH tunnel active? (if used)
- [ ] Camera connected? (Raspberry Pi)
- [ ] Port 5555 reachable?
- [ ] "VIDEO_READY" signal in log? (GUI output)
- [ ] "channel 5: open failed" in SSH tunnel → Normal, should disappear

### Control Not Working

- [ ] TCP port 10223 reachable?
- [ ] Server receiving commands? (check journalctl logs)
- [ ] PCA9685 detected? (i2cdetect -y 1)
- [ ] Movement commands visible in server log?

### Second Connect Not Working

- [ ] Latest version from git? (Reconnection fix)
- [ ] Check server logs: Shows "waiting for connection"?
- [ ] 2 second wait after disconnect?

### High CPU Load

- [ ] Latest version? (CPU optimization fix)
- [ ] FPV thread running only once?
- [ ] Sleep statements in loops present?

---

## 14. Contacts and Resources

**Original Manufacturer**: Adeept
- Website: www.adeept.com
- E-mail: support@adeept.com

**GitHub Repository**: https://github.com/schmiereck/Adeept_RaspClaws.git

**Developer**: schmiereck

**Original Author** (Adeept base code): William (2018/08/22)

---

## 15. Version History

| Date | Version | Changes |
|-------|---------|----------|
| 2026-01-17 | 2.0 | Reconnection Fix, CPU Optimization, Smooth Cam |
| 2026-01-16 | 1.5 | PCA9685 Fix (0x40), SSH Tunnel Support |
| 2026-01-16 | 1.4 | ZMQ PUB/SUB Fix, Video Stream Stabilization |
| 2018-08-22 | 1.0 | Original Adeept Release |

---

**Last Update**: 2026-01-20

**Status**: ✅ Production - Client can connect multiple times, video stream stable
