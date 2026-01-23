# ROS 2 Integration - Quick Start Guide

## Was wurde erstellt?

### 1. ROSServer.py
**Hauptdatei**: `Server/ROSServer.py`

ROS 2 Humble Node mit:
- **Topics**:
  - `/raspclaws/cmd_vel` (Twist) - Bewegungssteuerung
  - `/raspclaws/head_cmd` (Point) - Kopfbewegung
  - `/raspclaws/battery` (Float32) - Batteriespannung
  - `/raspclaws/cpu_usage` (Float32) - CPU Auslastung
  - `/raspclaws/status` (String) - Status

- **Services**:
  - `/raspclaws/reset_servos` - Servos zurücksetzen
  - `/raspclaws/set_smooth_mode` - Smooth Movement
  - `/raspclaws/set_smooth_cam` - Smooth Camera

### 2. Docker Setup
- `Dockerfile.ros2` - ros-humble-ros-base
- `docker-compose.ros2.yml` - Container Konfiguration
- `start_ros2.sh` - Management Script

### 3. Test Scripts
- `test_ros2_client.sh` - Client Tests (PC/Jetson)

### 4. Dokumentation
- `Docu/ROS2_INTEGRATION_DE.md` - Vollständige Anleitung

---

## Erste Schritte auf dem Pi

```bash
# 1. Docker installieren (falls nicht vorhanden)
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER
# Neu einloggen

# 2. Projekt auf Pi
cd /home/pi/adeept_raspclaws

# 3. Docker Image bauen (10-15 Min)
chmod +x start_ros2.sh
./start_ros2.sh build

# 4. ROS 2 Server starten
./start_ros2.sh start

# 5. Logs ansehen
./start_ros2.sh logs
```

---

## Erste Tests

### Test 1: Node läuft
```bash
./start_ros2.sh test
```

### Test 2: Bewegung
```bash
# Im Docker Container oder auf Pi mit ROS 2:
ros2 topic pub --once /raspclaws/cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.3}, angular: {z: 0.0}}"
```

### Test 3: Service
```bash
ros2 service call /raspclaws/reset_servos std_srvs/Trigger
```

---

## PC/Jetson Setup

```bash
# ROS 2 Humble installieren
sudo apt install ros-humble-desktop

# Source
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# ROS_DOMAIN_ID (muss gleich sein wie auf Pi!)
export ROS_DOMAIN_ID=0

# Node sehen
ros2 node list
# Sollte /raspclaws_node zeigen

# Teleop testen
sudo apt install ros-humble-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/raspclaws/cmd_vel
```

---

## Nächste Schritte

### Phase 1: Basis-Test (AKTUELL)
1. Docker auf Pi aufsetzen
2. Image bauen und starten
3. Von PC verbinden
4. Erste Bewegungstests

### Phase 2: Sensor-Integration
1. Camera Topic (sensor_msgs/Image)
2. Batterie-Monitoring
3. IMU Daten (falls MPU6050 funktioniert)

### Phase 3: Navigation
1. Odometry berechnen
2. TF Tree aufbauen
3. Navigation2 Stack
4. Cosmos Reason 2 Integration

---

## Hilfe

**Vollständige Dokumentation**: `Docu/ROS2_INTEGRATION_DE.md`

**Probleme?**
- Container Logs: `./start_ros2.sh logs`
- Container Status: `./start_ros2.sh status`
- In Container: `./start_ros2.sh shell`

**Node nicht gefunden?**
- ROS_DOMAIN_ID gleich? (Pi und PC)
- Firewall? `sudo ufw allow from 192.168.2.0/24`
- Multicast funktioniert?

---

## Wichtige Befehle

```bash
# Pi (Docker Management)
./start_ros2.sh start    # Starten
./start_ros2.sh stop     # Stoppen
./start_ros2.sh logs     # Logs
./start_ros2.sh test     # Connectivity Test

# PC (ROS 2 Client)
ros2 node list           # Nodes sehen
ros2 topic list          # Topics sehen
ros2 topic echo /raspclaws/status  # Topic mitlesen
ros2 service list        # Services sehen

# Bewegung
ros2 topic pub --once /raspclaws/cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.0}}"
```

---

**Status**: ✅ Bereit für erste Tests  
**Nächster Schritt**: Docker auf Pi installieren und Image bauen  
**Ziel**: Cosmos Reason 2 & ROS Planner Integration
