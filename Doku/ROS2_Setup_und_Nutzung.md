# ROS2 Setup und Nutzung - RaspClaws Roboter

**Letzte Aktualisierung:** 5. Februar 2026

## Übersicht

Dieses Dokument beschreibt die vollständige Einrichtung und Nutzung des ROS2-Systems für den RaspClaws-Roboter mit Multi-Machine-Setup.

### System-Architektur

```
┌────────────────────────────────────────┐
│  raspclaws-1 (192.168.2.126)          │
│  ┌──────────────────────────────────┐  │
│  │ GUIServer (system-Python 3.13)   │  │
│  │ - Picamera2                      │  │
│  │ - ZMQ Stream (Port 5555)         │  │
│  └──────────────┬───────────────────┘  │
│                 │ base64-encoded JPEG  │
│  ┌──────────────▼───────────────────┐  │
│  │ ROSServer (RoboStack Python 3.11)│  │
│  │ - ZMQ Bridge                     │  │
│  │ - ROS2 Topics Publisher          │  │
│  └──────────────┬───────────────────┘  │
└─────────────────┼──────────────────────┘
                  │ ROS2 DDS (Domain 1)
┌─────────────────▼──────────────────────┐
│  ubuntu1 (192.168.2.133)               │
│  - ROS2 Humble (native)                │
│  - Visualisierung: rviz2, rqt          │
│  - Steuerung: ros2 topic pub/echo      │
└────────────────────────────────────────┘
```

---

## 1. Einrichtung (einmalig)

### 1.1 ROS_DOMAIN_ID konfigurieren

**Auf beiden Raspberry Pis:**

```bash
# Bereits konfiguriert in ~/.bashrc:
# raspclaws-1: export ROS_DOMAIN_ID=1
# ubuntu1:     export ROS_DOMAIN_ID=1
```

✅ Bereits erledigt - keine Aktion nötig.

### 1.2 Skripte bereitstellen

```bash
# Auf raspclaws-1
cd /home/pi/Adeept_RaspClaws
git pull

# Skripte ausführbar machen
chmod +x start_rosserver.sh activate_camera.sh test_ros2_camera.sh
```

---

## 2. Tägliche Nutzung

### 2.1 Server starten (auf raspclaws-1)

**Terminal 1: GUIServer starten**

```bash
ssh pi@192.168.2.126
cd /home/pi/Adeept_RaspClaws/Server
python3 GUIServer.py
```

**Terminal 2: ROSServer starten**

```bash
ssh pi@192.168.2.126
cd /home/pi/Adeept_RaspClaws
./start_rosserver.sh
```

**Wichtig:** Lass beide Terminals offen und laufen!

### 2.2 Kamera aktivieren

**Option A: Über Web-GUI (empfohlen)**

1. Öffne Browser: `http://192.168.2.126:5000`
2. Klicke auf **"Camera Pause/Resume"** Button
3. Kamera sollte jetzt aktiv sein (Stream läuft)

**Option B: Über ROS2-Service**

```bash
# Von ubuntu1 oder raspclaws-1
ssh ubuntu@192.168.2.133
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=1

# Kamera aktivieren
./activate_camera.sh
```

⚠️ **Hinweis:** Option B aktiviert nur die ROS2-Seite. Die Kamera im GUIServer muss über Web-GUI aktiviert werden!

---

## 3. Testen und Nutzen

### 3.1 Topics anzeigen

**Von ubuntu1:**

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=1

# Alle Topics anzeigen
ros2 topic list

# Nur RaspClaws-Topics
ros2 topic list | grep raspclaws

# Verfügbare Topics:
# /raspclaws/battery
# /raspclaws/camera/camera_info
# /raspclaws/camera/image_compressed
# /raspclaws/camera/image_raw
# /raspclaws/cmd_vel
# /raspclaws/cpu_temp
# /raspclaws/cpu_usage
# /raspclaws/gyro_data
# /raspclaws/head_cmd
# /raspclaws/imu
# /raspclaws/ram_usage
# /raspclaws/servo_positions
# /raspclaws/status
```

### 3.2 Kamera-Stream prüfen

```bash
# Frame-Rate anzeigen (sollte ~30 Hz sein)
ros2 topic hz /raspclaws/camera/image_compressed

# Erwartete Ausgabe:
# average rate: 29.985
#   min: 0.032s max: 0.035s std dev: 0.00081s window: 31

# Bandwidth prüfen
ros2 topic bw /raspclaws/camera/image_compressed

# Kamera-Info anzeigen
ros2 topic echo /raspclaws/camera/camera_info --once
```

### 3.3 Sensor-Daten live anzeigen

```bash
# Batterie-Spannung
ros2 topic echo /raspclaws/battery

# IMU-Daten (MPU6050)
ros2 topic echo /raspclaws/imu

# CPU-Temperatur
ros2 topic echo /raspclaws/cpu_temp

# Robot-Status
ros2 topic echo /raspclaws/status
```

---

## 4. Visualisierung mit X-Windows

### 4.1 X-Server auf Windows installieren

**Wähle eine Option:**

- **VcXsrv** (empfohlen): https://sourceforge.net/projects/vcxsrv/
  - Starten: XLaunch → "Multiple windows" → "Start no client" → "Disable access control"

- **MobaXterm** (all-in-one): https://mobaxterm.mobatek.net/
  - X-Server startet automatisch

- **Xming**: https://sourceforge.net/projects/xming/

### 4.2 SSH mit X-Forwarding

```bash
# Von Windows-PC
ssh -X ubuntu@192.168.2.133

# Auf ubuntu1
export DISPLAY=localhost:10.0  # Falls nicht automatisch gesetzt
```

### 4.3 ROS2-Visualisierungstools starten

**rqt_image_view - Kamera-Bild anzeigen:**

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=1

rqt_image_view
```

Dann im Fenster:
- Topic auswählen: `/raspclaws/camera/image_raw` oder `/raspclaws/camera/image_compressed`

**rqt_graph - Topic-Graph visualisieren:**

```bash
rqt_graph
```

Zeigt alle Nodes, Topics und ihre Verbindungen.

**rviz2 - 3D-Visualisierung:**

```bash
rviz2
```

In rviz2:
1. **Add** → **By topic** → `/raspclaws/camera/image_raw` → **Image**
2. **Fixed Frame** auf `camera_link` setzen

**rqt_console - Log-Konsole:**

```bash
rqt_console
```

Zeigt alle ROS2-Log-Nachrichten.

---

## 5. Roboter steuern

### 5.1 Bewegungssteuerung

**Vorwärts bewegen:**

```bash
ros2 topic pub /raspclaws/cmd_vel geometry_msgs/Twist "linear: {z: 0.5}"
# z: 0.5 = 50% Geschwindigkeit vorwärts
```

**Rückwärts bewegen:**

```bash
ros2 topic pub /raspclaws/cmd_vel geometry_msgs/Twist "linear: {z: -0.5}"
# z: -0.5 = 50% Geschwindigkeit rückwärts
```

**Nach rechts drehen:**

```bash
ros2 topic pub /raspclaws/cmd_vel geometry_msgs/Twist "angular: {y: 0.5}"
# y: 0.5 = 50% Geschwindigkeit rechts
```

**Nach links drehen:**

```bash
ros2 topic pub /raspclaws/cmd_vel geometry_msgs/Twist "angular: {y: -0.5}"
```

**Bogen fahren (Forward + Right):**

```bash
ros2 topic pub /raspclaws/cmd_vel geometry_msgs/Twist "linear: {z: 0.5}, angular: {y: 0.3}"
```

**Stoppen:**

```bash
ros2 topic pub /raspclaws/cmd_vel geometry_msgs/Twist "linear: {z: 0.0}, angular: {y: 0.0}"
```

### 5.2 Kamera-Servo steuern

**Kamera nach links drehen:**

```bash
ros2 topic pub /raspclaws/head_cmd geometry_msgs/Point "x: -0.5, y: 0.0"
# x: -0.5 = links, +0.5 = rechts
```

**Kamera nach oben:**

```bash
ros2 topic pub /raspclaws/head_cmd geometry_msgs/Point "x: 0.0, y: 0.5"
# y: 0.5 = oben, -0.5 = unten
```

**Kamera zentrieren:**

```bash
ros2 topic pub /raspclaws/head_cmd geometry_msgs/Point "x: 0.0, y: 0.0"
```

### 5.3 Services nutzen

**Servos zurücksetzen:**

```bash
ros2 service call /raspclaws/reset_servos std_srvs/srv/Trigger
```

**Smooth-Mode aktivieren:**

```bash
ros2 service call /raspclaws/set_smooth_mode std_srvs/srv/SetBool "data: true"
```

**Servo-Standby (Servos ausschalten):**

```bash
ros2 service call /raspclaws/set_servo_standby std_srvs/srv/SetBool "data: true"
```

**Servo-Wakeup (Servos einschalten):**

```bash
ros2 service call /raspclaws/set_servo_standby std_srvs/srv/SetBool "data: false"
```

---

## 6. Troubleshooting

### 6.1 Keine Topics sichtbar von ubuntu1

**Problem:** `ros2 topic list` zeigt keine `/raspclaws/*` Topics

**Lösung:**

```bash
# 1. Prüfe ROS_DOMAIN_ID
echo $ROS_DOMAIN_ID  # Muss "1" sein

# 2. Prüfe Netzwerk-Verbindung
ping 192.168.2.126

# 3. Prüfe ob ROSServer läuft
ssh pi@192.168.2.126 "ps aux | grep ROSServer"

# 4. Prüfe ROSServer Log
ssh pi@192.168.2.126 "tail -30 /tmp/rosserver.log"
```

### 6.2 Kamera-Topics existieren, aber keine Frames

**Problem:** `ros2 topic hz /raspclaws/camera/image_compressed` zeigt nichts

**Lösung:**

1. **Kamera im GUIServer aktivieren** (Web-GUI!)
2. **Prüfe GUIServer-Status:**
   ```bash
   ssh pi@192.168.2.126 "tail -20 /tmp/guiserver.log | grep camera"
   ```
   Sollte zeigen: `Camera stream RESUMED`

3. **Prüfe ROSServer ZMQ-Bridge:**
   ```bash
   ssh pi@192.168.2.126 "tail -50 /tmp/rosserver.log | grep -E 'camera|ZMQ|frame'"
   ```
   Sollte zeigen: `✓ Receiving frames from GUIServer`

4. **Test-Skript ausführen:**
   ```bash
   ssh pi@192.168.2.126
   cd /home/pi/Adeept_RaspClaws
   ./test_ros2_camera.sh
   ```

### 6.3 "Failed to decode frame from ZMQ" Fehler

**Problem:** ROSServer zeigt Dekodierungs-Fehler

**Ursache:** GUIServer sendet base64-encoded JPEG, aber Dekodierung schlägt fehl

**Lösung:**

```bash
# Stelle sicher, dass die neueste Version läuft
cd /home/pi/Adeept_RaspClaws
git pull

# ROSServer neu starten
pkill -f "python3.*ROSServer"
./start_rosserver.sh
```

Der Fix (base64-Dekodierung) ist in Server/FPV_ROS2.py seit Commit `bed1f4f`.

### 6.4 ROSServer startet nicht

**Problem:** `./start_rosserver.sh` funktioniert nicht

**Lösung:**

```bash
# 1. Prüfe micromamba
which micromamba  # Sollte /usr/local/bin/micromamba zeigen

# 2. Prüfe ros_env
micromamba env list  # ros_env sollte existieren

# 3. Manueller Start
export MAMBA_EXE='/usr/local/bin/micromamba'
export MAMBA_ROOT_PREFIX='/home/pi/micromamba'
eval "$($MAMBA_EXE shell hook --shell bash --root-prefix $MAMBA_ROOT_PREFIX)"
micromamba activate ros_env
export ROS_DOMAIN_ID=1
cd /home/pi/Adeept_RaspClaws/Server
python3 ROSServer.py
```

### 6.5 X-Windows funktioniert nicht

**Problem:** `rqt_image_view` zeigt Fehler oder öffnet kein Fenster

**Lösung:**

```bash
# 1. Prüfe DISPLAY-Variable
echo $DISPLAY  # Sollte z.B. "localhost:10.0" sein

# 2. Teste X-Forwarding
xclock  # Sollte eine Uhr anzeigen

# 3. Prüfe X-Server auf Windows
# - VcXsrv: Läuft das XLaunch-Icon im Systemtray?
# - MobaXterm: Zeigt das Dashboard "X server: RUNNING"?

# 4. SSH mit X-Forwarding neu verbinden
# Von Windows:
ssh -X ubuntu@192.168.2.133
```

---

## 7. Fortgeschrittene Nutzung

### 7.1 Eigene ROS2-Nodes schreiben

**Beispiel: Einfacher Subscriber in Python:**

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')
        self.subscription = self.create_subscription(
            Float32,
            '/raspclaws/battery',
            self.battery_callback,
            10)

    def battery_callback(self, msg):
        voltage = msg.data
        self.get_logger().info(f'Battery: {voltage:.2f}V')

        if voltage < 6.5:
            self.get_logger().warn('⚠️ Low battery!')

def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Ausführen:**

```bash
# ROS_DOMAIN_ID muss gesetzt sein
export ROS_DOMAIN_ID=1
python3 battery_monitor.py
```

### 7.2 Daten aufzeichnen mit rosbag

```bash
# Alle Topics aufzeichnen
ros2 bag record -a

# Nur Kamera aufzeichnen
ros2 bag record /raspclaws/camera/image_compressed /raspclaws/camera/camera_info

# Bestimmte Topics aufzeichnen
ros2 bag record /raspclaws/battery /raspclaws/imu /raspclaws/cmd_vel

# Aufzeichnung wiedergeben
ros2 bag play <bag_file>
```

### 7.3 Launch-Files erstellen

**Beispiel: raspclaws_monitor.launch.py**

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='camera_view',
            arguments=['/raspclaws/camera/image_raw']
        ),
        Node(
            package='rqt_console',
            executable='rqt_console',
            name='console'
        ),
    ])
```

**Ausführen:**

```bash
ros2 launch raspclaws_monitor.launch.py
```

---

## 8. Nützliche Kommandos

### 8.1 Schnellreferenz

```bash
# ROS2-Umgebung laden
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=1

# Topics
ros2 topic list                              # Alle Topics
ros2 topic echo /topic_name                  # Topic-Daten live
ros2 topic hz /topic_name                    # Frequenz
ros2 topic bw /topic_name                    # Bandwidth
ros2 topic info /topic_name                  # Topic-Info

# Nodes
ros2 node list                               # Alle Nodes
ros2 node info /node_name                    # Node-Info

# Services
ros2 service list                            # Alle Services
ros2 service call /service_name Type "args"  # Service aufrufen

# Messages/Types
ros2 interface show geometry_msgs/Twist      # Message-Definition
ros2 interface list                          # Alle verfügbaren Types
```

### 8.2 Hilfsskripte

```bash
# Auf raspclaws-1
cd /home/pi/Adeept_RaspClaws

# ROSServer starten
./start_rosserver.sh

# Kamera aktivieren
./activate_camera.sh

# Kamera-System testen
./test_ros2_camera.sh
```

---

## 9. Systemkonfiguration

### 9.1 Wichtige Dateien

| Datei | Zweck |
|-------|-------|
| `Server/GUIServer.py` | Web-GUI und Picamera2 |
| `Server/FPV.py` | Kamera-Capture und ZMQ-Publisher |
| `Server/ROSServer.py` | ROS2-Node (Main) |
| `Server/FPV_ROS2.py` | ZMQ-to-ROS2 Bridge |
| `start_rosserver.sh` | ROSServer-Startup-Skript |
| `activate_camera.sh` | Kamera-Aktivierungs-Skript |
| `test_ros2_camera.sh` | Test-Skript |
| `/tmp/guiserver.log` | GUIServer-Log |
| `/tmp/rosserver.log` | ROSServer-Log |

### 9.2 Netzwerk-Ports

| Port | Service | Protokoll |
|------|---------|-----------|
| 5000 | Web-GUI | HTTP |
| 5555 | ZMQ Video Stream | ZMQ PUB/SUB |
| 10223 | Command Server | TCP |
| 7400-7500 | ROS2 DDS | UDP Multicast |

### 9.3 ROS2-Konfiguration

```bash
# raspclaws-1 und ubuntu1
export ROS_DOMAIN_ID=1           # ROS2 Domain
# DDS: Default (CycloneDDS oder FastDDS)
# Discovery: Multicast (automatisch)
```

---

## 10. Referenzen

- **ROS2 Humble Dokumentation:** https://docs.ros.org/en/humble/
- **RoboStack:** https://robostack.github.io/
- **Picamera2 Dokumentation:** https://datasheets.raspberrypi.com/camera/picamera2-manual.pdf
- **ZMQ Dokumentation:** https://zeromq.org/

---

**Bei Fragen oder Problemen:**
- Prüfe die Log-Dateien: `/tmp/guiserver.log`, `/tmp/rosserver.log`
- Führe Test-Skript aus: `./test_ros2_camera.sh`
- Prüfe project_info.md für aktuelle System-Status
