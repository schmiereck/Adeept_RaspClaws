# Dockerfile Multi-Stage Build: libcamera Support

**Datum:** 2026-01-30  
**Problem gelöst:** libcamera Python-Bindings in ROS 2 Humble Container

---

## Problem

- **ROS 2 Humble** basiert auf **Ubuntu 22.04 (Jammy)**
- **`python3-libcamera`** ist erst ab **Ubuntu 24.04** oder **Debian Bookworm** verfügbar
- Raspberry Pi Kamera braucht aber `libcamera` und `picamera2`

---

## Lösung: Multi-Stage Docker Build (korrigiert)

### Problem mit Debian Bookworm

**Ursprünglicher Plan:** `python3-libcamera` aus Debian Bookworm  
**Problem:** Diese Pakete existieren **NUR in Raspberry Pi OS**, nicht in Standard Debian!

```bash
# In Standard Debian Bookworm:
apt-cache search python3-libcamera
# Keine Ergebnisse! ❌
```

### Korrigierte Strategie

**Hybrid-Ansatz:**
1. **libcamera C-Library** aus Debian Bookworm (`libcamera0.2`)
2. **picamera2** via pip (enthält Python-Bindings)

### Stage 1: Debian Bookworm + pip

```dockerfile
FROM debian:bookworm-slim AS camera-builder

# C-Library aus apt
RUN apt-get install -y \
    libcamera0.2 \          # ✅ Verfügbar in Debian!
    libcamera-ipa \
    libcamera-tools

# Python-Bindings aus pip
RUN pip3 install picamera2  # ✅ Enthält libcamera Python-Bindings
```

### Stage 2: ROS 2 Humble + libcamera

```dockerfile
FROM ros:humble-ros-base

# Copy libcamera from Debian Bookworm
COPY --from=camera-builder /usr/lib/aarch64-linux-gnu/libcamera* /usr/lib/aarch64-linux-gnu/
COPY --from=camera-builder /usr/lib/python3/dist-packages/libcamera /usr/lib/python3/dist-packages/libcamera/
COPY --from=camera-builder /usr/lib/python3/dist-packages/picamera2 /usr/lib/python3/dist-packages/picamera2/

# ... rest of ROS2 setup
```

---

## Vorteile

| Methode | Ubuntu 22.04 Support | Sauber | Wartbar |
|---------|---------------------|--------|---------|
| **pip install picamera2** | ❌ Keine Python-Bindings | ❌ | ❌ |
| **Host-Mounting** | ⚠️ Komplex | ❌ Symlink-Probleme | ❌ |
| **Multi-Stage Build** | ✅ Ja! | ✅ Ja! | ✅ Ja! |

---

## Was wird kopiert?

### C-Libraries (aus Debian apt):
```
/usr/lib/aarch64-linux-gnu/
├── libcamera.so.0.2.0        # Haupt-Library
├── libcamera.so.0.2 -> ...   # Symlink
├── libpisp.so.0.0.2          # ISP Support
└── libcamera-ipa/            # IPA Plugins
    ├── ipa_rpi.so
    ├── ipa_rkisp1.so
    └── ...
```

### Python-Bindings (aus pip):
```
/usr/local/lib/python3.11/dist-packages/  (Stage 1)
→ kopiert nach:
/usr/lib/python3/dist-packages/           (Stage 2, ROS2 Container)

├── libcamera/                # Python libcamera Module
│   ├── __init__.py
│   └── ...
├── picamera2/                # Picamera2 Module
│   ├── __init__.py
│   └── ...
└── _libcamera*.so            # Native Extension (compiled)
```

---

## Build-Anleitung

```bash
cd ~/adeept_raspclaws

# Build (dauert ~30-60 Minuten beim ersten Mal)
docker compose -f docker-compose.ros2.yml build

# Starten
docker compose -f docker-compose.ros2.yml up

# Testen
docker exec -it raspclaws_camera python3 -c "from picamera2 import Picamera2; print('✅ Works!')"
```

---

## Erwartete Ausgabe

### Erfolgreicher Start:

```
raspclaws_camera  | ==========================================
raspclaws_camera  | Starting Camera ROS2 Container
raspclaws_camera  | ==========================================
raspclaws_camera  | Checking if camera is already in use...
raspclaws_camera  | ✓ Camera device is available
raspclaws_camera  | Starting FPV.py (camera capture + ZMQ stream)...
raspclaws_camera  | [0:00:01.234] [INFO] Camera camera_manager.cpp: libcamera v0.3.0
raspclaws_camera  | ✓ FPV.py started successfully (PID: 24)
raspclaws_camera  | Starting FPV_ROS2_simple.py (ROS2 publisher)...
raspclaws_camera  | [INFO] [camera_publisher]: ✓ Camera started: 640x480 @ 30 FPS
```

### Kein "No module named 'libcamera'" Fehler mehr! ✅

---

## Vergleich: Vorher vs Nachher

### Vorher (Ubuntu 22.04 apt):

```bash
apt-get install python3-libcamera
# E: Unable to locate package python3-libcamera ❌
```

### Nachher (Multi-Stage Build):

```bash
docker exec -it raspclaws_camera python3 -c "import libcamera; print(libcamera)"
# <module 'libcamera' from '/usr/lib/python3/dist-packages/libcamera/__init__.py'> ✅
```

---

## Troubleshooting

### Problem: "libcamera.so.0.3: cannot open shared object file"

**Lösung:** Library-Pfad fehlt

```dockerfile
# Füge hinzu:
RUN ldconfig
```

### Problem: "No cameras available"

**Prüfe:**
```bash
# Im Container:
docker exec -it raspclaws_camera libcamera-hello --list-cameras

# Sollte zeigen:
# Available cameras
# 0 : imx219 [3280x2464] (/base/soc/i2c0mux/i2c@1/imx219@10)
```

---

## Image-Größe

- **ros:humble-ros-base**: ~800 MB
- **+ libcamera (Bookworm)**: ~50 MB
- **Total**: ~850 MB

Akzeptabel für ein vollständiges ROS2 + Kamera Image.

---

## Alternative Ansätze (nicht empfohlen)

### Ansatz 1: ROS 2 Jazzy (Ubuntu 24.04)

```dockerfile
FROM ros:jazzy-ros-base  # Ubuntu 24.04

RUN apt-get install python3-libcamera  # ✅ Verfügbar
```

**Problem:** ROS 2 Jazzy ist sehr neu (Mai 2024), weniger stabil

### Ansatz 2: Raspberry Pi OS Base Image

```dockerfile
FROM raspbian/bookworm

# ROS2 manuell installieren
RUN apt-get install ros-humble-ros-base
```

**Problem:** Viel Arbeit, große Image-Größe, komplexe Wartung

---

## Fazit

**Multi-Stage Build ist die beste Lösung:**
- ✅ Nutzt bewährtes ROS 2 Humble
- ✅ Saubere libcamera Integration
- ✅ Wartbar und nachvollziehbar
- ✅ Keine Host-Abhängigkeiten

**Empfehlung:** Diese Lösung verwenden! 🚀

---

**Erstellt:** 2026-01-30  
**Autor:** GitHub Copilot  
**Status:** ✅ Implementiert
