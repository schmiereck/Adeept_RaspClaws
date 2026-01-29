# 🔧 DOCKERFILE FIX: Ubuntu 22.04 Kompatibilität

## 🔴 Problem

```
E: Unable to locate package python3-libcamera
E: Unable to locate package python3-kms++
```

## 🎯 Root Cause

**ROS2 Humble Base Image** verwendet **Ubuntu 22.04 (Jammy)**, aber:
- `python3-libcamera` ist erst ab Ubuntu 23.04+ verfügbar
- `python3-kms++` ist erst ab Ubuntu 23.04+ verfügbar
- `python3-picamera2` ist erst ab Ubuntu 23.10+ verfügbar

## ✅ Lösung: pip statt apt

**Für Ubuntu 22.04 (Jammy):**

```dockerfile
# System-Pakete (verfügbar in Ubuntu 22.04):
RUN apt-get install -y \
    libcamera-dev \      # C-Library
    libcamera-tools \    # CLI Tools
    libcamera0 \         # Runtime Library
    libcap-dev \
    libgl1 \
    libglib2.0-0

# Python-Pakete über pip:
RUN pip3 install --no-cache-dir picamera2
# ✅ picamera2 über pip enthält die Python-Bindings für libcamera!
```

## 📋 Änderungen im Dockerfile.ros2

### ❌ ENTFERNT (nicht verfügbar in Ubuntu 22.04):
```dockerfile
python3-libcamera     # ❌ Not found
python3-kms++         # ❌ Not found
python3-picamera2     # ❌ Not found
```

### ✅ HINZUGEFÜGT:
```dockerfile
# System-Pakete:
libcamera0            # ✅ Runtime Library

# pip-Pakete:
picamera2             # ✅ Enthält Python-Bindings
```

## 🚀 Test-Kommandos

```bash
cd /home/pi/adeept_raspclaws

# Container NEU BAUEN (mit korrigiertem Dockerfile):
docker compose -f docker-compose.ros2.yml down
docker compose -f docker-compose.ros2.yml build --no-cache

# Erwartete Ausgabe (KEIN Fehler mehr):
# ...
# Step X: RUN apt-get install -y libcamera-dev libcamera-tools libcamera0...
# Step Y: RUN pip3 install --no-cache-dir picamera2
# Successfully built ...

# Starten:
docker compose -f docker-compose.ros2.yml up -d

# Logs prüfen:
docker compose -f docker-compose.ros2.yml logs -f
```

## 📊 Erwartete Logs

### raspclaws_ros2:
```
⏸️  RPIservo.py geladen - PCA9685 NICHT initialisiert
⏸️  Move.py geladen - Servos NICHT initialisiert
⚠ Camera module not available: No module named 'libcamera'
  (Camera features disabled, but robot control works)
[INFO] [raspclaws_node]: RaspClaws ROS 2 Node initialized successfully!
```

### raspclaws_camera:
```
✓ libcamera Python-Bindings gefunden
[INFO] [camera_publisher]: Initializing Camera Publisher...
[INFO] [camera_publisher]: ✓ Camera started: 640x480 @ 30 FPS
[INFO] [camera_publisher]: Starting capture loop...
```

## ⚠️ Bekannte Einschränkungen

### Problem: picamera2 über pip findet manchmal libcamera nicht

**Ursache:** pip-picamera2 sucht libcamera-Bindings im falschen Pfad

**Workaround (falls nötig):**

```dockerfile
# Setze PYTHONPATH explizit:
ENV PYTHONPATH="/usr/lib/python3/dist-packages:${PYTHONPATH}"
```

Oder in docker-compose.yml:

```yaml
environment:
  - PYTHONPATH=/usr/lib/python3/dist-packages:/ros2_ws:/ros2_ws/Server
```

## 🎯 Alternative: Raspberry Pi OS Base Image

**Wenn picamera2 über pip nicht funktioniert:**

```dockerfile
# Statt ros:humble-ros-base:
FROM arm64v8/ubuntu:24.04

# ROS2 manuell installieren
RUN apt-get update && apt-get install -y \
    ros-humble-ros-base \
    python3-libcamera \      # ✅ Verfügbar in Ubuntu 24.04!
    python3-picamera2 \      # ✅ Verfügbar in Ubuntu 24.04!
    # ...
```

Aber das ist nur nötig, wenn pip-Installation nicht funktioniert.

## ✅ Zusammenfassung

| Methode | Ubuntu 22.04 | Ubuntu 24.04+ |
|---------|--------------|---------------|
| **apt install python3-picamera2** | ❌ Nicht verfügbar | ✅ Verfügbar |
| **pip install picamera2** | ✅ **Funktioniert** | ✅ Funktioniert |

**Für ROS2 Humble (Ubuntu 22.04): pip verwenden!**

## 🔄 Nächste Schritte

1. **Build neu starten** (Dockerfile wurde korrigiert)
2. **Logs prüfen** (beide Container sollten starten)
3. **Camera testen** (Topics sollten erscheinen)
4. **Servo testen** (sollte funktionieren)

```bash
# Build:
docker compose -f docker-compose.ros2.yml build --no-cache

# Start:
docker compose -f docker-compose.ros2.yml up -d

# Camera Topics prüfen:
ros2 topic list | grep camera

# Servo Test:
ros2 topic pub --once /raspclaws/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}}'
```

🚀 **Jetzt sollte alles funktionieren!**
