# 🔧 HOTFIX: libcamera ModuleNotFoundError

## Problem
```
raspclaws_camera  | ModuleNotFoundError: No module named 'libcamera'
```

## Ursache

**`picamera2` über pip installiert** kann die System-Python-Bindings nicht finden!

```
pip install picamera2  ❌
  └─ Installiert picamera2
  └─ Sucht libcamera Python-Bindings
  └─ Findet sie NICHT (weil sie im System-Python-Pfad liegen)
```

## Lösung

**Installiere alles über apt (nicht pip):**

```dockerfile
# Dockerfile.ros2 - System-Pakete:
RUN apt-get install -y \
    python3-libcamera \      # Python-Bindings für libcamera
    python3-kms++ \          # Display-System
    python3-picamera2        # Picamera2 mit korrekten Bindings
```

## Änderungen im Dockerfile.ros2

### ✅ HINZUGEFÜGT (apt):
```dockerfile
# Kamera und Grafik-Abhängigkeiten
RUN apt-get update && apt-get install -y \
    libcamera-dev \
    libcamera-tools \
    python3-libcamera \      # ✅ NEU
    python3-kms++ \          # ✅ NEU
    libcap-dev \
    libgl1 \
    libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# ROS2 spezifische System-Pakete
RUN apt-get update && apt-get install -y \
    ros-humble-cv-bridge \
    python3-psutil \
    python3-picamera2 \      # ✅ NEU (über apt statt pip!)
    && rm -rf /var/lib/apt/lists/*
```

### ❌ ENTFERNT (pip):
```dockerfile
# Hardware-Treiber und spezialisierte Libs
RUN pip3 install --no-cache-dir \
    adafruit-pca9685 \
    adafruit-gpio \
    mpu6050-raspberrypi \
    smbus2 \
    gpiozero \
    lgpio \
    imutils \
    psutil
    # picamera2  ❌ ENTFERNT (wird jetzt über apt installiert)
```

## Test-Kommandos

```bash
# 1. Container NEU BAUEN (wichtig!)
cd /home/pi/adeept_raspclaws
docker compose -f docker-compose.ros2.yml down
docker compose -f docker-compose.ros2.yml build --no-cache

# 2. Starten
docker compose -f docker-compose.ros2.yml up -d

# 3. Logs prüfen (libcamera Fehler sollte weg sein!)
docker compose -f docker-compose.ros2.yml logs raspclaws_camera

# Erwartete Ausgabe (KEIN Fehler mehr):
# [INFO] [camera_publisher]: Initializing Camera Publisher...
# [INFO] [camera_publisher]: ✓ Camera started: 640x480 @ 30 FPS

# 4. Topics prüfen
docker exec -it raspclaws_ros2 bash -c "source /opt/ros/humble/setup.bash && ros2 topic list | grep camera"

# Erwartete Ausgabe:
# /raspclaws/camera/camera_info
# /raspclaws/camera/image_compressed
# /raspclaws/camera/image_raw
```

## Warum apt statt pip?

| Methode | Ergebnis |
|---------|----------|
| **pip install picamera2** | ❌ Findet `libcamera` nicht (falscher Python-Pfad) |
| **apt install python3-picamera2** | ✅ Korrekt mit System-Python-Bindings verlinkt |

**System-Python-Pakete (`python3-*`)** sind für Hardware-nahe Bibliotheken **immer besser** als pip!

## Quick Fix Zusammenfassung

```bash
# Dockerfile.ros2 ändern:
# 1. Hinzufügen: python3-libcamera, python3-kms++, python3-picamera2
# 2. Entfernen: picamera2 aus pip

# Container neu bauen:
docker compose -f docker-compose.ros2.yml build --no-cache

# Starten:
docker compose -f docker-compose.ros2.yml up -d

# ✅ Fertig!
```

## Weitere Hinweise

- **Immer `--no-cache` verwenden** beim Rebuild nach Dockerfile-Änderungen
- System-Pakete (`python3-*`) verwenden für: GPIO, I2C, Kamera, Hardware
- pip verwenden für: Reine Python-Bibliotheken ohne Hardware-Zugriff
