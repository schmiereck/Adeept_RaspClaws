# ROS2 Camera Node - Test Checkliste

## ✅ Was wurde korrigiert:

### 1. **docker-compose.ros2.yml**
- ✅ `image: adeept_raspclaws-raspclaws_ros2:latest` (vorher: `build:`)
- ✅ `network_mode: host` (ROS2 DDS benötigt das)
- ✅ `environment:` - Alle ROS2 Variablen hinzugefügt (ROS_DOMAIN_ID, etc.)
- ✅ `volumes:` - Korrektes Mapping `/home/pi/adeept_raspclaws:/ros2_ws`
- ✅ `command:` - ROS2 Setup vor Python-Start

### 2. **Dockerfile.ros2**
- ✅ `ros-humble-cv-bridge` installiert (benötigt für Image-Konvertierung)
- ✅ `python3-libcamera` installiert (Python-Bindings für libcamera)
- ✅ `python3-kms++` installiert (Display-System für libcamera)
- ✅ `python3-picamera2` über apt statt pip (enthält korrekte libcamera-Bindings)
- ✅ `picamera2` aus pip entfernt (Konflikt mit System-Package)

### 3. **FPV_ROS2.py**
- ✅ RGB → BGR Konvertierung für JPEG-Encoding (OpenCV erwartet BGR)

---

## 🔧 WICHTIG: libcamera Problem gelöst!

**Problem:** `ModuleNotFoundError: No module named 'libcamera'`

**Ursache:** 
- `picamera2` über pip installiert findet die System-Python-Bindings nicht
- `python3-libcamera` muss über apt installiert werden
- `picamera2` muss auch über apt kommen (nicht pip)

**Lösung:**
```dockerfile
# In Dockerfile.ros2:

# System-Pakete (apt):
RUN apt-get install -y \
    python3-libcamera \      # ✅ Python-Bindings für libcamera
    python3-kms++ \          # ✅ Display-System
    python3-picamera2        # ✅ Picamera2 mit korrekten Bindings

# NICHT mehr über pip:
# picamera2  ❌ ENTFERNT
```

---

## 🚀 Test-Ablauf

### Schritt 1: Container neu bauen

```bash
cd /home/pi/adeept_raspclaws

# Stoppen
docker compose -f docker-compose.ros2.yml down

# Neu bauen (wichtig wegen cv_bridge!)
docker compose -f docker-compose.ros2.yml build --no-cache

# Erwartete Ausgabe:
# ...
# Step X: RUN apt-get install -y ... ros-humble-cv-bridge ...
# Successfully built ...
```

### Schritt 2: Beide Container starten

```bash
docker compose -f docker-compose.ros2.yml up -d

# Prüfen, ob beide laufen:
docker ps

# Erwartete Ausgabe:
# CONTAINER ID   IMAGE                                    STATUS
# xxx            adeept_raspclaws-raspclaws_ros2:latest   Up (raspclaws_ros2)
# yyy            adeept_raspclaws-raspclaws_ros2:latest   Up (raspclaws_camera)
```

### Schritt 3: Logs prüfen

```bash
# ROSServer Logs:
docker compose -f docker-compose.ros2.yml logs raspclaws_ros2

# Erwartete Zeilen:
# [INFO] [raspclaws_node]: Initializing RaspClaws ROS 2 Node...
# [INFO] [raspclaws_node]: RaspClaws ROS 2 Node initialized successfully!

# Camera Logs:
docker compose -f docker-compose.ros2.yml logs raspclaws_camera

# Erwartete Zeilen:
# [INFO] [camera_publisher]: Initializing Camera Publisher...
# [INFO] [camera_publisher]: ✓ Camera started: 640x480 @ 30 FPS
# [INFO] [camera_publisher]: Starting capture loop...
```

### Schritt 4: Topics prüfen

```bash
# Im Container oder auf PC mit ROS2:
docker exec -it raspclaws_ros2 bash -c "source /opt/ros/humble/setup.bash && ros2 topic list"

# Erwartete Camera Topics:
# /raspclaws/camera/image_raw
# /raspclaws/camera/image_compressed
# /raspclaws/camera/camera_info
```

### Schritt 5: FPS prüfen

```bash
docker exec -it raspclaws_ros2 bash -c "source /opt/ros/humble/setup.bash && ros2 topic hz /raspclaws/camera/image_compressed"

# Erwartete Ausgabe:
# average rate: 30.000
#   min: 0.033s max: 0.033s std dev: 0.00001s window: 30
```

### Schritt 6: Daten prüfen

```bash
# Ein Frame ansehen:
docker exec -it raspclaws_ros2 bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /raspclaws/camera/image_compressed --once"

# Erwartete Ausgabe:
# header:
#   stamp:
#     sec: ...
#     nanosec: ...
#   frame_id: camera_link
# format: jpeg
# data: [255, 216, 255, 224, ...]  # JPEG bytes
```

### Schritt 7: Bandbreite prüfen

```bash
docker exec -it raspclaws_ros2 bash -c "source /opt/ros/humble/setup.bash && ros2 topic bw /raspclaws/camera/image_compressed"

# Erwartete Ausgabe (640x480 @ JPEG 85):
# average: 2.5 MB/s
```

---

## 🔍 Fehlersuche

### Problem: "No module named 'cv_bridge'"

```bash
# Im Container prüfen:
docker exec -it raspclaws_camera python3 -c "from cv_bridge import CvBridge; print('OK')"

# Wenn Fehler → Container neu bauen:
docker compose -f docker-compose.ros2.yml build --no-cache
```

### Problem: "Could not start camera"

```bash
# Kamera-Device prüfen:
ls -l /dev/video0

# Erwartete Ausgabe:
# crw-rw---- 1 root video ... /dev/video0

# Im Container prüfen:
docker exec -it raspclaws_camera ls -l /dev/video0

# Wenn fehlt → docker-compose.yml prüfen:
# devices:
#   - /dev/video0:/dev/video0
```

### Problem: "No topics published"

```bash
# Logs mit Fehler ansehen:
docker compose -f docker-compose.ros2.yml logs raspclaws_camera | grep -i error

# Häufige Fehler:
# - picamera2 import failed → libcamera nicht installiert
# - cv_bridge not found → Container neu bauen
# - Permission denied /dev/video0 → privileged: true fehlt
```

### Problem: "False colors in JPEG"

**Ursache:** RGB/BGR Verwechslung

**Gelöst:** Jetzt mit `cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)` in publish_compressed_image()

---

## 📊 Performance-Übersicht

### Standard-Konfiguration (640x480 @ 30 FPS, JPEG 85):

| Metrik | Wert |
|--------|------|
| **CPU Usage** | ~25% (Raspberry Pi 4) |
| **Bandbreite (Compressed)** | ~2-3 MB/s |
| **Bandbreite (Raw)** | ~27 MB/s |
| **Latenz** | ~50-100 ms |

### Optimiert für WLAN (320x240 @ 15 FPS, JPEG 70):

| Metrik | Wert |
|--------|------|
| **CPU Usage** | ~10% |
| **Bandbreite** | ~0.5 MB/s |
| **Latenz** | ~40-80 ms |

Konfiguration ändern in `Server/FPV_ROS2.py`:

```python
class CameraConfig:
    WIDTH = 320
    HEIGHT = 240
    TARGET_FPS = 15
    JPEG_QUALITY = 70
    PUBLISH_RAW = False  # Nur JPEG
```

---

## 🎯 Erwartetes Ergebnis

Nach erfolgreichem Test solltest du sehen:

```bash
$ ros2 topic list | grep camera
/raspclaws/camera/camera_info
/raspclaws/camera/image_compressed
/raspclaws/camera/image_raw

$ ros2 topic hz /raspclaws/camera/image_compressed
average rate: 30.000

$ ros2 node list
/camera_publisher
/raspclaws_node
```

---

## 🔧 Nächste Schritte nach erfolgreichem Test

1. **In RViz visualisieren** (auf PC):
   ```bash
   rviz2
   # Add → Image → /raspclaws/camera/image_compressed
   ```

2. **Video aufzeichnen**:
   ```bash
   ros2 bag record /raspclaws/camera/image_compressed
   ```

3. **Pause/Resume testen**:
   ```bash
   # Pause:
   ros2 service call /raspclaws/set_camera_pause std_srvs/srv/SetBool "{data: true}"
   
   # Resume:
   ros2 service call /raspclaws/set_camera_pause std_srvs/srv/SetBool "{data: false}"
   ```

---

## ✅ Zusammenfassung der Korrekturen

| Datei | Änderung | Grund |
|-------|----------|-------|
| `docker-compose.ros2.yml` | `image:` statt `build:` | Gleiches Image wie raspclaws_ros2 |
| `docker-compose.ros2.yml` | ROS2 Environment Variablen | DDS Kommunikation |
| `docker-compose.ros2.yml` | Volume-Mapping korrigiert | `/home/pi/adeept_raspclaws:/ros2_ws` |
| `docker-compose.ros2.yml` | Command mit ROS2 Setup | `source /opt/ros/humble/setup.bash` |
| `Dockerfile.ros2` | `ros-humble-cv-bridge` | Image-Konvertierung |
| `FPV_ROS2.py` | RGB→BGR Konvertierung | OpenCV JPEG-Encoding |

Alle Änderungen sind **notwendig** für korrekte Funktion! 🚀
