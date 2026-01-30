# Kamera-Container Fix: "Pipeline handler in use"

## Problem

Der Kamera-Container meldet, dass die Kamera nicht verfügbar ist, obwohl sie mit GUIServer funktioniert:

```
Camera __init__ sequence did not complete.
Pipeline handler in use by another process
```

## Ursache

**Die Kamera kann nur von EINEM Prozess gleichzeitig verwendet werden:**

- ✅ GUIServer (auf dem Host) verwendet die Kamera
- ❌ Docker-Container kann NICHT gleichzeitig zugreifen

## Lösung: GUIServer stoppen BEVOR Docker gestartet wird

### Schritt 1: GUIServer stoppen

```bash
# Auf dem Raspberry Pi:
cd ~/adeept_raspclaws

# Alle GUIServer-Prozesse beenden
sudo bash Server/stop_guiserver.sh
```

**Ausgabe:**
```
==========================================
Stopping GUIServer and FPV processes
==========================================
Found GUIServer processes: 1234
  Killing process 1234...
✓ All GUIServer processes stopped
==========================================
```

### Schritt 2: Docker-Container neu starten

```bash
# Container neu bauen (nur beim ersten Mal oder nach Code-Änderungen)
docker compose -f docker-compose.ros2.yml build

# Container starten
docker compose -f docker-compose.ros2.yml up
```

**Erwartete Ausgabe:**
```
raspclaws_camera  | Checking if camera is already in use...
raspclaws_camera  | ✓ Camera device is available
raspclaws_camera  | Starting FPV.py (camera capture + ZMQ stream)...
raspclaws_camera  | Waiting 3 seconds for FPV.py to initialize...
raspclaws_camera  | ✓ FPV.py started successfully (PID: 123)
raspclaws_camera  | Starting FPV_ROS2_simple.py (ROS2 publisher)...
```

### Schritt 3: Testen

```bash
# In einem anderen Terminal:
docker exec -it raspclaws_ros2 bash -c "source /opt/ros/humble/setup.bash && ros2 topic list | grep camera"

# Erwartete Ausgabe:
# /raspclaws/camera/camera_info
# /raspclaws/camera/image_compressed
```

## Verbesserte Fehlerdiagnose

Das Start-Script prüft jetzt automatisch, ob die Kamera blockiert ist:

### Wenn GUIServer noch läuft:

```
⚠️  WARNING: Camera is already in use by another process:
COMMAND    PID  USER   FD   TYPE DEVICE SIZE/OFF NODE NAME
python3   1234  root   5u   CHR  81,0      0t0  123 /dev/video0

This is usually caused by:
  - GUIServer running on the host
  - Another camera application

To fix this:
  1. Stop GUIServer: bash Server/stop_guiserver.sh
  2. Restart this container: docker compose restart raspclaws_camera

Container will sleep to prevent restart loop
```

### Kamera erfolgreich initialisiert:

```
✓ Camera device is available
Starting FPV.py (camera capture + ZMQ stream)...
Waiting 3 seconds for FPV.py to initialize...
✓ FPV.py started successfully (PID: 123)
```

## Wichtige Änderungen

### 1. docker-compose.ros2.yml

Jetzt werden **alle** benötigten Kamera-Devices gemappt:

```yaml
devices:
  - /dev/video0:/dev/video0   # V4L2 camera
  - /dev/video10:/dev/video10 # ISP device
  - /dev/video11:/dev/video11
  # ... weitere ISP devices
  - /dev/media0:/dev/media0   # Media controller
  - /dev/media1:/dev/media1
  # ...

volumes:
  - /dev:/dev         # Alle Devices
  - /sys:/sys         # Hardware-Erkennung
  - /run/udev:/run/udev:ro  # udev rules
```

### 2. start_camera_ros2.sh

Prüft jetzt, ob die Kamera bereits verwendet wird:

```bash
if lsof /dev/video0 2>/dev/null | grep -q video0; then
    echo "⚠️  WARNING: Camera is already in use by another process"
    # ... Hilfreiche Fehlermeldung ...
    sleep infinity  # Verhindert Restart-Loop
fi
```

### 3. Dockerfile.ros2

`lsof` wurde hinzugefügt für Kamera-Zugriffs-Prüfung:

```dockerfile
RUN apt-get install -y \
    lsof \
    # ... andere Pakete
```

## Workflow: GUIServer vs Docker

### Option 1: GUIServer verwenden (normale Entwicklung)

```bash
# Docker stoppen
docker compose -f docker-compose.ros2.yml down

# GUIServer starten
sudo python3 Server/GUIServer.py

# GUI-Client verbinden
python Client/GUI.py
```

### Option 2: Docker/ROS2 verwenden

```bash
# GUIServer stoppen
sudo bash Server/stop_guiserver.sh

# Docker starten
docker compose -f docker-compose.ros2.yml up

# ROS2-Befehle senden
ros2 topic pub /raspclaws/cmd_vel geometry_msgs/Twist "{linear: {z: 0.5}}"
```

## Troubleshooting

### Problem: "Camera is already in use"

**Lösung:**
```bash
# Finde alle Prozesse, die Kamera verwenden
sudo lsof /dev/video0

# Stoppe sie
sudo bash Server/stop_guiserver.sh

# Container neu starten
docker compose -f docker-compose.ros2.yml restart raspclaws_camera
```

### Problem: Container startet sofort neu

**Ursache:** Alte Version des Scripts ohne `sleep infinity`

**Lösung:**
```bash
# Code aktualisieren
git pull

# Container neu bauen
docker compose -f docker-compose.ros2.yml build

# Container neu starten
docker compose -f docker-compose.ros2.yml up
```

### Problem: libcamera Fehler im Container

**Überprüfen:**
```bash
# Teste libcamera auf dem Host
libcamera-hello

# Prüfe Devices
ls -l /dev/video* /dev/media*

# Container-Logs
docker compose -f docker-compose.ros2.yml logs raspclaws_camera
```

## Best Practice

**Vor Docker-Start immer GUIServer stoppen:**

```bash
#!/bin/bash
# start_ros2.sh

cd ~/adeept_raspclaws

# 1. Stoppe alte Prozesse
sudo bash Server/stop_guiserver.sh

# 2. Warte kurz
sleep 1

# 3. Starte Docker
docker compose -f docker-compose.ros2.yml up
```

Speichere das als `start_ros2.sh` und mache es ausführbar:
```bash
chmod +x start_ros2.sh
./start_ros2.sh
```

---

**Problem behoben:** 2026-01-30  
**Autor:** GitHub Copilot  
**Hauptursache:** GUIServer und Docker-Container können Kamera nicht gleichzeitig verwenden
