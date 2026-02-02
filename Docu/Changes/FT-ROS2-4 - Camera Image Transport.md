# FT-ROS2-4: Camera Image Transport mit Compression

**Status:** ✅ Implementiert  
**Datum:** 2026-02-02  
**Kategorie:** ROS2 Integration

## 📋 Übersicht

Integration von `image_transport` für komprimierte Kameraübertragung im ROS2-Stack. Das System publiziert jetzt sowohl Raw-Bilder (`/image_raw`) als auch komprimierte Bilder (`/image_raw/compressed`).

## 🎯 Ziel

- **Effiziente Bildübertragung**: Komprimierte JPEG-Bilder für Netzwerkübertragung
- **Flexibilität**: Clients können zwischen Raw und Compressed wählen
- **Standardkonform**: Nutzt ROS2 `image_transport` Plugin-System

## 🔧 Implementierung

### Architektur

```
┌─────────────────┐
│  v4l2_camera    │
│  /dev/video14   │
└────────┬────────┘
         │ /image_raw (sensor_msgs/Image)
         │
         ├──────────────────────────┐
         │                          │
         v                          v
┌─────────────────┐        ┌──────────────────┐
│  Raw Topic      │        │  image_transport │
│  /image_raw     │        │  republish       │
└─────────────────┘        └────────┬─────────┘
                                    │
                                    v
                           ┌──────────────────────┐
                           │  Compressed Topic    │
                           │  /image_raw/compressed│
                           │  (JPEG, Quality 85)  │
                           └──────────────────────┘
```

### Komponenten

#### 1. Dockerfile.ros2

Installiert benötigte Pakete:
```dockerfile
ros-humble-v4l2-camera              # Kamera-Treiber
ros-humble-image-transport          # Transport-Framework
ros-humble-image-transport-plugins  # Plugin-Sammlung (inkl. compressed_image_transport)
```

**Hinweis:** `compressed_image_transport` ist bereits in `image-transport-plugins` enthalten.

#### 2. camera.launch.py

Startet zwei Nodes:

**v4l2_camera_node:**
- Liest von `/dev/video14` (bcm2835-isp)
- Publiziert `/image_raw` (sensor_msgs/Image, RGB8)
- Publiziert `/camera_info`

**image_republisher:**
- Subscribes: `/image_raw`
- Publishes: `/image_raw/compressed` (sensor_msgs/CompressedImage)
- Format: JPEG mit Quality 85
- Nutzt `image_transport republish` Tool

### Parameter

```yaml
Kamera:
  - video_device: /dev/video14
  - image_size: [640, 480]
  - pixel_format: YUYV
  - output_encoding: rgb8
  - io_method: mmap

Compression:
  - compressed.format: jpeg
  - compressed.jpeg_quality: 85
```

## 📡 Topics

| Topic | Typ | Frequenz | Beschreibung |
|-------|-----|----------|--------------|
| `/image_raw` | sensor_msgs/Image | 30 Hz | Unkomprimiertes RGB8-Bild |
| `/image_raw/compressed` | sensor_msgs/CompressedImage | 30 Hz | JPEG-komprimiert (85%) |
| `/camera_info` | sensor_msgs/CameraInfo | 30 Hz | Kamera-Kalibrierung |

## 🧪 Test

### Auf dem Raspberry Pi:

```bash
# Topics auflisten
docker exec -it raspclaws_camera bash -c \
  "source /opt/ros/humble/setup.bash && ros2 topic list"

# Raw-Image prüfen
docker exec -it raspclaws_camera bash -c \
  "source /opt/ros/humble/setup.bash && ros2 topic hz /image_raw"

# Compressed-Image prüfen
docker exec -it raspclaws_camera bash -c \
  "source /opt/ros/humble/setup.bash && ros2 topic hz /image_raw/compressed"

# Bandwidth vergleichen
docker exec -it raspclaws_camera bash -c \
  "source /opt/ros/humble/setup.bash && ros2 topic bw /image_raw"
docker exec -it raspclaws_camera bash -c \
  "source /opt/ros/humble/setup.bash && ros2 topic bw /image_raw/compressed"
```

### Auf dem PC (WSL):

```bash
# Compressed-Image ansehen (bandbreitensparend)
ros2 run rqt_image_view rqt_image_view /image_raw/compressed

# Oder Raw-Image
ros2 run rqt_image_view rqt_image_view /image_raw
```

## 📊 Performance

| Format | Größe pro Frame | Bandwidth (30 FPS) |
|--------|----------------|-------------------|
| Raw RGB8 (640x480) | ~900 KB | ~27 MB/s |
| JPEG Quality 85 | ~30-50 KB | ~1-1.5 MB/s |

**Einsparung:** ~95% weniger Netzwerk-Traffic

## 🔄 Vergleich zu alter Implementierung

### Vorher (FPV_ROS2.py):
- ❌ picamera2 in Container nicht verfügbar
- ❌ Keine libcamera-Python-Bindings
- ❌ Komplex zu debuggen

### Jetzt (v4l2_camera + image_transport):
- ✅ Standard ROS2-Pakete
- ✅ Funktioniert in Container
- ✅ Automatische Compression
- ✅ Standard-konform (sensor_msgs)

## 🐛 Troubleshooting

### "No module named 'picamera2'"
→ **Lösung:** Nicht mehr relevant, v4l2_camera nutzt kein Python

### "Failed stream start: Invalid argument (22)"
→ **Lösung:** Falsches `/dev/video*` Device
→ **Fix:** Nutze `/dev/video14` statt `/dev/video0`

### "WARNING: topic [/image_raw/compressed] does not appear to be published yet"
→ **Lösung:** Prüfe ob `image_republisher` Node läuft:
```bash
docker exec -it raspclaws_camera bash -c \
  "source /opt/ros/humble/setup.bash && ros2 node list"
```

### Compression Quality anpassen

In `camera.launch.py`:
```python
'compressed.jpeg_quality': 85  # 0-100, höher = bessere Qualität
```

## 🎓 Referenzen

- [ROS2 image_transport](http://wiki.ros.org/image_transport)
- [v4l2_camera Package](https://github.com/ros-drivers/v4l2_camera)
- [CompressedImage Message](https://docs.ros2.org/latest/api/sensor_msgs/msg/CompressedImage.html)

## 🔍 Paket-Verfügbarkeit

Die folgenden Pakete sind **standardmäßig** in ROS Humble (Ubuntu 22.04 Jammy) verfügbar:

```bash
# Diese Pakete werden installiert:
ros-humble-image-transport              ✅ Standard
ros-humble-image-transport-plugins      ✅ Standard (enthält compressed_image_transport)
ros-humble-v4l2-camera                  ✅ Standard
```

**Verifiziert auf Raspberry Pi:**
```bash
$ docker run --rm ros:humble-ros-base bash -c \
  "apt-cache search ros-humble-image-transport | grep -E 'image-transport'"

# Output:
ros-humble-image-transport
ros-humble-image-transport-plugins
```

**Keine zusätzlichen Repositories erforderlich!**

## ✅ Erfolg

Mit diesem Setup haben wir:
1. ✅ Funktionierende Kamera im Docker-Container
2. ✅ Standard-konforme ROS2-Topics
3. ✅ Automatische JPEG-Kompression
4. ✅ ~95% weniger Netzwerk-Bandwidth
5. ✅ Keine Python-picamera2-Abhängigkeit mehr
