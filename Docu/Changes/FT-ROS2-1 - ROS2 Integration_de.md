# FT-ROS2-1: ROS 2 Integration für Adeept RaspClaws

## Übersicht

ROS 2 Integration für den RaspClaws Roboter mit folgender Architektur:

```
┌─────────────────────────────────────────────────┐
│             PC / Jetson Orin                    │
│  ┌──────────────────┐  ┌──────────────────┐    │
│  │ Cosmos Reason 2  │  │  ROS 2 Planner   │    │
│  │  (Navigation)    │  │  (Path Planning) │    │
│  └──────────────────┘  └──────────────────┘    │
│             │                    │              │
│             └────────┬───────────┘              │
│                      │ ROS 2 DDS                │
└──────────────────────┼──────────────────────────┘
                       │ (Network)
┌──────────────────────┼──────────────────────────┐
│        Raspberry Pi  │                          │
│  ┌───────────────────▼────────────────────┐    │
│  │  Docker Container (ros-humble-base)    │    │
│  │  ┌──────────────────────────────────┐  │    │
│  │  │      ROSServer.py                │  │    │
│  │  │  - Topics: cmd_vel, battery      │  │    │
│  │  │  - Services: reset, smooth_mode  │  │    │
│  │  └──────────────────────────────────┘  │    │
│  └─────────────────────┬───────────────────┘    │
│                        │                         │
│  ┌─────────────────────▼───────────────────┐    │
│  │  Hardware (I2C, SPI, GPIO)              │    │
│  │  - PCA9685 (Servos)                     │    │
│  │  - WS2812 (LEDs)                        │    │
│  │  - Camera                               │    │
│  └─────────────────────────────────────────┘    │
└─────────────────────────────────────────────────┘
```

---

## Installation auf Raspberry Pi

### 1. Docker installieren

```bash
# Docker installieren (falls nicht vorhanden)
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# User zur docker Gruppe hinzufügen
sudo usermod -aG docker $USER
newgrp docker

# Neustart oder neu einloggen
# Dann Docker testen:
docker --version
```

### 2. Docker Compose installieren

```bash
# (NICHT machen!)
sudo apt-get update
sudo apt-get install -y docker-compose

# ODER neuere Version (NICHT machen!):
sudo pip3 install docker-compose


# ODER ganz neuer Version:
# Deine neuen Befehle für den Raspberry Pi:

# Bauen:
# Gehe in dein Projektverzeichnis
cd /home/pi/adeept_raspclaws
docker compose -f docker-compose.ros2.yml build

# Starten:
docker compose -f docker-compose.ros2.yml up -d

# Logs:
docker compose -f docker-compose.ros2.yml logs -f

```

### 3. Projekt auf Pi übertragen

```bash
# Auf dem Pi:
cd /home/pi
git clone https://github.com/schmiereck/Adeept_RaspClaws.git
cd Adeept_RaspClaws
```

Oder via SCP von Windows:
```powershell
scp -r C:\Users\SCMJ178\IdeaProjects\Adeept_RaspClaws pi@192.168.2.126:/home/pi/
```

### 4. Docker Container bauen

```bash
cd /home/pi/Adeept_RaspClaws

# Image bauen (dauert ~10-15 Minuten)
docker-compose -f docker-compose.ros2.yml build

# Container starten
docker-compose -f docker-compose.ros2.yml up -d

# Logs ansehen
docker-compose -f docker-compose.ros2.yml logs -f
```

### 5. USB-Tests ohne Akku (Lazy Initialization)

Der ROSServer verwendet **Lazy Initialization**:
- **Beim Start:** Servos werden NICHT initialisiert → Servos bleiben weich
- **Bei erstem Befehl:** Servos werden automatisch initialisiert

**Vorteil:** Du kannst über USB testen, ohne dass Servos steif werden. Erst wenn ein Bewegungsbefehl kommt, wird die Hardware initialisiert.

```bash
# Einfach starten (USB-Strom reicht):
sudo python3 Server/ROSServer.py

# Ausgabe:
# 💤 Lazy initialization enabled - hardware will be initialized on first command
#    (Servos stay soft until first movement/head command)
```

Bei erstem Bewegungsbefehl:
```bash
ros2 topic pub /raspclaws/cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}}"

# Auf dem Pi:
# 🤖 Initializing robot hardware on first command...
# ✓ Robot hardware initialized successfully
```

**Details:** Siehe `Server/ROSServer_USB_TEST_MODE.md`

---

## ROS 2 Topics & Services

### Published Topics

| Topic | Message Type | Beschreibung |
|-------|--------------|--------------|
| `/raspclaws/battery` | `std_msgs/Float32` | Batteriespannung in Volt |
| `/raspclaws/cpu_usage` | `std_msgs/Float32` | CPU Auslastung in % |
| `/raspclaws/status` | `std_msgs/String` | Status-Nachrichten |

### Subscribed Topics

| Topic | Message Type | Beschreibung |
|-------|--------------|--------------|
| `/raspclaws/cmd_vel` | `geometry_msgs/Twist` | Bewegungsbefehle (linear.x, angular.z) |
| `/raspclaws/head_cmd` | `geometry_msgs/Point` | Kopfbewegungen (x=left/right, y=up/down) |

### Services

| Service | Type | Beschreibung |
|---------|------|--------------|
| `/raspclaws/reset_servos` | `std_srvs/Trigger` | Alle Servos auf Home-Position |
| `/raspclaws/set_smooth_mode` | `std_srvs/SetBool` | Smooth Movement Mode ein/aus |
| `/raspclaws/set_smooth_cam` | `std_srvs/SetBool` | Smooth Camera Mode ein/aus |

---

## Erste Tests

### Test 1: Node läuft

```bash
# Auf dem Pi (im Container oder außerhalb):
ros2 node list

# Erwartete Ausgabe:
# /raspclaws_node
```

### Test 2: Topics ansehen

```bash
# Liste aller Topics
ros2 topic list

# Topic ausgeben
ros2 topic echo /raspclaws/status
```

### Test 3: Bewegungsbefehl senden

```bash
# Vorwärts fahren
ros2 topic pub --once /raspclaws/cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"

# Stopp
ros2 topic pub --once /raspclaws/cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"

# Linksdrehung
ros2 topic pub --once /raspclaws/cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}"
```

### Test 4: Kopfbewegung

```bash
# Kopf nach rechts
ros2 topic pub --once /raspclaws/head_cmd geometry_msgs/Point "{x: 0.5, y: 0.0, z: 0.0}"

# Kopf hoch
ros2 topic pub --once /raspclaws/head_cmd geometry_msgs/Point "{x: 0.0, y: 0.5, z: 0.0}"

# Kopf zurück (Home)
ros2 service call /raspclaws/reset_servos std_srvs/Trigger
```

### Test 5: Smooth Mode

```bash
# Smooth Mode aktivieren
ros2 service call /raspclaws/set_smooth_mode std_srvs/SetBool "{data: true}"

# Smooth Mode deaktivieren
ros2 service call /raspclaws/set_smooth_mode std_srvs/SetBool "{data: false}"
```

---

## PC/Jetson Setup (ROS 2 Client)

### 1. ROS 2 Humble installieren

**Ubuntu 22.04**:
```bash
# Setup Sources
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble Desktop
sudo apt update
sudo apt install ros-humble-desktop -y

# Source ROS 2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Netzwerk-Konfiguration

```bash
# ROS_DOMAIN_ID muss auf PC und Pi gleich sein!
export ROS_DOMAIN_ID=0

# Für permanente Konfiguration:
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
```

### 3. Test: Nodes sehen

```bash
# Auf dem PC:
ros2 node list

# Sollte raspclaws_node vom Pi anzeigen!
```

### 4. Test: Teleop

```bash
# Teleop Keyboard installieren
sudo apt install ros-humble-teleop-twist-keyboard

# Starten
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/raspclaws/cmd_vel
```

---

## Test-Client ohne Cosmos/Planner (Einfacher)

Für schnelle Tests ohne Installation von Cosmos Reason 2 oder ROS Planner gibt es einen einfachen Python Test-Client.

### Installation

```bash
# ROS 2 Humble muss installiert sein
source /opt/ros/humble/setup.bash

# Python Dependencies (meist schon vorhanden)
pip3 install rclpy
```

### Verwendung

**Einzelne Befehle**:
```bash
# Liste aller Befehle
python3 ros2_test_client.py list

# Bewegung
python3 ros2_test_client.py forward
python3 ros2_test_client.py forward --speed 0.3
python3 ros2_test_client.py left
python3 ros2_test_client.py stop

# Kopf
python3 ros2_test_client.py head_left
python3 ros2_test_client.py head_up
python3 ros2_test_client.py head_center

# Services
python3 ros2_test_client.py reset
python3 ros2_test_client.py smooth_on
python3 ros2_test_client.py smooth_off

# Monitoring
python3 ros2_test_client.py status
python3 ros2_test_client.py battery
python3 ros2_test_client.py monitor --duration 30
```

**Interaktiver Modus**:
```bash
python3 ros2_test_client.py interactive
# oder einfach:
python3 ros2_test_client.py

# Im interaktiven Modus:
raspclaws> forward
raspclaws> status
raspclaws> head_left
raspclaws> stop
raspclaws> quit
```

### Verfügbare Befehle

**Bewegung**:
- `forward [--speed 0.5]` - Vorwärts
- `backward [--speed 0.5]` - Rückwärts
- `left [--speed 0.5]` - Links drehen
- `right [--speed 0.5]` - Rechts drehen
- `stop` - Stoppen

**Kopf**:
- `head_left` - Kopf nach links
- `head_right` - Kopf nach rechts
- `head_up` - Kopf nach oben
- `head_down` - Kopf nach unten
- `head_center` - Kopf zentrieren

**Services**:
- `reset` - Alle Servos auf Home-Position
- `smooth_on` / `smooth_off` - Smooth Movement Mode
- `smooth_cam_on` / `smooth_cam_off` - Smooth Camera Mode

**Monitoring**:
- `status` - Aktuellen Status anzeigen
- `battery` - Batteriespannung anzeigen
- `monitor [--duration 10]` - Alle Topics überwachen

---

## Troubleshooting

### Problem: Node wird nicht gefunden

**Ursache**: ROS 2 DDS Discovery funktioniert nicht

**Lösung**:
```bash
# 1. Firewall prüfen (auf Pi und PC)
sudo ufw allow from 192.168.2.0/24

# 2. ROS_DOMAIN_ID prüfen
echo $ROS_DOMAIN_ID  # Muss auf beiden gleich sein

# 3. Multicast testen
# Auf Pi:
iperf3 -s -B 239.255.0.1

# Auf PC:
iperf3 -c 239.255.0.1 -u
```

### Problem: I2C Permission Denied im Container

**Lösung**:
```bash
# Auf dem Pi (außerhalb Container):
sudo chmod 666 /dev/i2c-1
```

Oder in `docker-compose.ros2.yml` ändern:
```yaml
privileged: true  # Bereits gesetzt
```

### Problem: Container startet nicht

```bash
# Logs ansehen
docker-compose -f docker-compose.ros2.yml logs

# Container neu bauen
docker-compose -f docker-compose.ros2.yml down
docker-compose -f docker-compose.ros2.yml build --no-cache
docker-compose -f docker-compose.ros2.yml up -d
```

---

## Protocol Constants Integration

ROSServer.py verwendet die zentralen Command-Konstanten aus `protocol.py`:

```python
from protocol import (
    CMD_FORWARD, CMD_BACKWARD, CMD_LEFT, CMD_RIGHT,
    CMD_FAST, CMD_SLOW, MOVE_STAND,
    CMD_LOOK_UP, CMD_LOOK_DOWN, CMD_LOOK_LEFT, CMD_LOOK_RIGHT, CMD_LOOK_HOME,
    CMD_SMOOTH_CAM, CMD_SMOOTH_CAM_OFF
)
```

**Vorteile**:
- ✅ Konsistente Command-Strings zwischen Client und Server
- ✅ Vermeidet Tippfehler
- ✅ IDE Autocomplete-Unterstützung
- ✅ Einfachere Wartung

**Beispiel Twist-Callback**:
```python
def cmd_vel_callback(self, msg):
    linear_x = msg.linear.x
    angular_z = msg.angular.z

    if abs(linear_x) > 0.1:
        if linear_x > 0:
            move.commandInput(CMD_FORWARD)  # statt 'forward'
        else:
            move.commandInput(CMD_BACKWARD)
    elif abs(angular_z) > 0.1:
        if angular_z > 0:
            move.commandInput(CMD_LEFT)
        else:
            move.commandInput(CMD_RIGHT)
    else:
        move.commandInput(MOVE_STAND)
```

---

## Nächste Schritte

### Phase 1: Basis-Funktionalität (✅ AKTUELL)
- [x] ROSServer.py erstellt
- [x] ROSServer.py verwendet protocol.py Konstanten
- [x] Docker Setup
- [x] cmd_vel Topic
- [x] Basis Services
- [x] Test-Client (ros2_test_client.py)
- [ ] Tests auf Hardware mit echtem Roboter

### Phase 2: Sensor-Integration
- [ ] Camera Topic (`sensor_msgs/Image`)
- [ ] IMU Topic (falls MPU6050 funktioniert)
- [ ] Batterie-Überwachung aus `Voltage.py`
- [ ] CPU-Info aus `Info.py`

### Phase 3: Cosmos Reason 2 Integration
- [ ] Odometry Topic
- [ ] TF Transforms (base_link, camera_link)
- [ ] Navigation2 Stack
- [ ] Map Server

### Phase 4: ROS Planner Integration
- [ ] Path Planning Services
- [ ] Obstacle Avoidance
- [ ] Goal Handling

---

## Nützliche Befehle

```bash
# Container Management
docker-compose -f docker-compose.ros2.yml up -d      # Starten
docker-compose -f docker-compose.ros2.yml down       # Stoppen
docker-compose -f docker-compose.ros2.yml restart    # Neustart
docker-compose -f docker-compose.ros2.yml logs -f    # Logs folgen

# In Container einloggen
docker exec -it raspclaws_ros2 /bin/bash

# ROS 2 Node Info
ros2 node info /raspclaws_node

# Topic Info
ros2 topic info /raspclaws/cmd_vel
ros2 topic hz /raspclaws/battery

# Service auflisten
ros2 service list
ros2 service type /raspclaws/reset_servos
```

---

## Architektur-Diagramm

```
RaspClaws ROS 2 Node
├── Publishers (2Hz)
│   ├── /raspclaws/battery (Float32)
│   ├── /raspclaws/cpu_usage (Float32)
│   └── /raspclaws/status (String)
│
├── Subscribers
│   ├── /raspclaws/cmd_vel (Twist)
│   │   └→ Move.py: commandInput()
│   └── /raspclaws/head_cmd (Point)
│       └→ Move.py: look_up/down/left/right()
│
└── Services
    ├── /raspclaws/reset_servos (Trigger)
    │   └→ Move.py: home()
    ├── /raspclaws/set_smooth_mode (SetBool)
    │   └→ Move.py: commandInput('slow'/'fast')
    └── /raspclaws/set_smooth_cam (SetBool)
        └→ Move.py: commandInput('smoothCam'/'smoothCamOff')
```

---

## Geänderte Dateien

- **Server/ROSServer.py**: ROS 2 Server mit Topics, Services und protocol.py Integration
- **ros2_test_client.py**: Python Test-Client für einfache Tests
- **protocol.py**: Zentrale Command-Konstanten
- **docker-compose.ros2.yml**: Docker Setup (falls vorhanden)

---

**Feature ID**: FT-ros2-1
**Version**: 1.1
**Status**: ✅ Bereit für Hardware-Tests
**Datum**: 2026-01-20
