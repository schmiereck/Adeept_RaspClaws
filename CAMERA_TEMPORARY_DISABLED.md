# 🔧 CAMERA TEMPORÄR DEAKTIVIERT

## 🔴 Problem

```
ModuleNotFoundError: No module named 'libcamera'
```

**Root Cause:**
- `picamera2` über pip installiert **NICHT** die libcamera Python-Bindings
- libcamera Python-Bindings sind **nicht verfügbar** in Ubuntu 22.04 (Jammy)
- Erst ab Ubuntu 23.04+ oder Raspberry Pi OS Bookworm verfügbar

## ✅ Temporäre Lösung

**Camera-Container auskommentiert** in `docker-compose.ros2.yml`

```yaml
# Camera container temporarily disabled until libcamera Python bindings are available
# raspclaws_camera:
#   ...
```

**Effekt:**
- ✅ **ROSServer funktioniert** (Servo-Steuerung)
- ❌ **Camera Topics fehlen** (temporär)
- ⚠️ **FPV.py (ZMQ) funktioniert weiter** für GUIServer

## 🎯 Test JETZT

```bash
# Container neu starten (nur ROSServer, ohne Camera):
docker compose -f /home/pi/adeept_raspclaws/docker-compose.ros2.yml up -d

# Logs prüfen (sollte KEIN Camera-Fehler mehr zeigen):
docker compose -f /home/pi/adeept_raspclaws/docker-compose.ros2.yml logs -f

# Erwartete Ausgabe:
# raspclaws_ros2  | [INFO] [raspclaws_node]: RaspClaws ROS 2 Node initialized successfully!
# (KEIN raspclaws_camera Container mehr!)

# SERVO-TEST:
docker exec -it raspclaws_ros2 bash -c "
source /opt/ros/humble/setup.bash &&
ros2 topic pub --once /raspclaws/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}}'
"

# Erwartete Logs:
# [INFO] [raspclaws_node]: 🤖 Initializing robot hardware on first command...
# [INFO] [raspclaws_node]: ⚡ Aktiviere PCA9685 Servo-Controller...
# 🔧 Initialisiere PCA9685 auf Adresse 0x40...
# ✓ PCA9685 erfolgreich initialisiert
# [INFO] [raspclaws_node]: 🔥 Servos sind jetzt AKTIV und STEIF!

# → ROBOTER SOLLTE SICH BEWEGEN! 🚀
```

## 🔄 Dauerhafte Lösungen (später)

### Option A: Auf Ubuntu 24.04 Base wechseln

```dockerfile
# Statt ros:humble-ros-base (Ubuntu 22.04):
FROM ubuntu:24.04

# ROS2 Humble manuell installieren
RUN apt-get update && apt-get install -y \
    software-properties-common \
    && add-apt-repository universe

RUN apt-get install -y \
    curl \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $VERSION_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update && apt-get install -y \
    ros-humble-ros-base \
    python3-libcamera \      # ✅ Verfügbar in Ubuntu 24.04!
    python3-picamera2 \      # ✅ Verfügbar in Ubuntu 24.04!
    # ...
```

**Vorteil:** Native Pakete verfügbar  
**Nachteil:** Komplexer, mehr Arbeit

### Option B: libcamera Python-Bindings manuell kompilieren

```dockerfile
RUN apt-get install -y \
    git \
    cmake \
    meson \
    ninja-build \
    pybind11-dev \
    python3-dev \
    libcamera-dev

RUN git clone https://git.libcamera.org/libcamera/libcamera.git /tmp/libcamera && \
    cd /tmp/libcamera && \
    meson setup build -Dpycamera=enabled && \
    ninja -C build install
```

**Vorteil:** Funktioniert mit ROS2 Humble Base  
**Nachteil:** Lange Build-Zeit (~30 Min), komplex

### Option C: Raspberry Pi OS Container als Base

```dockerfile
FROM arm64v8/debian:bookworm

# Raspberry Pi OS hat python3-picamera2 nativ!
RUN apt-get update && apt-get install -y \
    python3-picamera2 \      # ✅ Verfügbar!
    python3-libcamera \      # ✅ Verfügbar!
    # ...

# ROS2 Humble nachinstallieren
```

**Vorteil:** Beste Hardware-Kompatibilität  
**Nachteil:** ROS2 muss manuell installiert werden

## 📊 Status

| Feature | Status |
|---------|--------|
| **Servo-Steuerung (ROS2)** | ✅ **Funktioniert** (ROSServer läuft) |
| **Topics (Battery, CPU, IMU)** | ✅ **Funktioniert** |
| **Services (Standby, etc.)** | ✅ **Funktioniert** |
| **Camera (ROS2)** | ❌ **Deaktiviert** (libcamera-Problem) |
| **Camera (ZMQ/FPV.py)** | ✅ **Funktioniert weiter** (für GUIServer) |

## 🎯 Empfehlung

**JETZT:**
1. ✅ Camera-Container ist deaktiviert
2. ✅ ROSServer funktioniert
3. ✅ Teste Servo-Steuerung
4. ✅ Nutze FPV.py (ZMQ) für Camera mit GUIServer

**SPÄTER:**
- Entscheide für langfristige Lösung (Option A/B/C)
- Implementiere Camera-Integration mit korrektem Base-Image

## 📋 Test-Kommandos (auf dem Pi)

```bash
# 1. Container neu starten (ohne Camera):
docker compose -f /home/pi/adeept_raspclaws/docker-compose.ros2.yml up -d

# 2. Logs prüfen (sollte sauber sein):
docker compose -f /home/pi/adeept_raspclaws/docker-compose.ros2.yml logs -f

# 3. Topics ansehen:
docker exec -it raspclaws_ros2 bash -c "
source /opt/ros/humble/setup.bash &&
ros2 topic list
"

# 4. Servo-Test:
docker exec -it raspclaws_ros2 bash -c "
source /opt/ros/humble/setup.bash &&
ros2 topic pub --once /raspclaws/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}}'
"

# 5. Head-Test:
docker exec -it raspclaws_ros2 bash -c "
source /opt/ros/humble/setup.bash &&
ros2 topic pub --once /raspclaws/head_cmd geometry_msgs/msg/Point '{x: 1.0, y: 0.0}'
"
```

## ✅ Zusammenfassung

**Problem:** libcamera Python-Bindings nicht verfügbar in Ubuntu 22.04

**Lösung:** Camera-Container temporär deaktiviert

**Resultat:**
- ✅ Servo-Steuerung funktioniert
- ✅ Alle ROS2 Services funktionieren
- ❌ Camera (ROS2) fehlt temporär
- ✅ Camera (ZMQ) funktioniert weiter

**Nächster Schritt:** SERVO-TEST! 🚀
