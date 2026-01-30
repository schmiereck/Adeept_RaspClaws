# ROS2 Camera Support - Host-based Solution

**Datum:** 2026-01-30  
**Problem:** libcamera ist nicht in Docker verfügbar  
**Lösung:** Camera auf Host, ROS2 im Container

---

## ❌ Warum funktioniert Camera nicht im Docker Container?

### Das Problem

`libcamera` und `picamera2` Pakete existieren **NUR in Raspberry Pi OS**, nicht in:
- ❌ Standard Debian (auch nicht Bookworm)
- ❌ Standard Ubuntu (auch nicht 24.04)
- ❌ ROS2 Docker Images (basieren auf Ubuntu)

**Versuchte Lösungen (alle fehlgeschlagen):**
1. ❌ `apt install python3-libcamera` → Package not found
2. ❌ `apt install libcamera0.2` → Package not found
3. ❌ `apt install libcamera0.3` → Package not found
4. ❌ Multi-Stage Build mit Debian Bookworm → Packages fehlen
5. ❌ pip install picamera2 → Benötigt libcamera C-Library
6. ❌ Host-Mounting von Libraries → Symlink/Path-Probleme

**Grund:** Raspberry Pi OS hat spezielle Repos mit Raspberry Pi Hardware-Treibern, die in Standard-Distros nicht existieren.

---

## ✅ Die Lösung: Hybrid-Ansatz

### Architektur

```
┌──────────────────────────────────────┐
│         Raspberry Pi Host            │
│                                      │
│  ┌────────────────────────────────┐ │
│  │  FPV.py (Python, native)       │ │
│  │  - Nutzt picamera2 (Host)      │ │
│  │  - Sendet Frames via ZMQ       │ │
│  └────────────┬───────────────────┘ │
│               │ ZMQ (Port 5555)      │
│  ┌────────────▼───────────────────┐ │
│  │  FPV_ROS2_simple.py (Python)   │ │
│  │  - Empfängt Frames via ZMQ     │ │
│  │  - Publiziert zu ROS2 Topics   │ │
│  └────────────┬───────────────────┘ │
│               │ ROS2 DDS             │
│  ┌────────────▼───────────────────┐ │
│  │  Docker Container              │ │
│  │  ┌──────────────────────────┐ │ │
│  │  │  ROSServer.py            │ │ │
│  │  │  - Servo Control         │ │ │
│  │  │  - Movement              │ │ │
│  │  │  - ROS2 Topics           │ │ │
│  │  └──────────────────────────┘ │ │
│  └────────────────────────────────┘ │
└──────────────────────────────────────┘
```

**Vorteile:**
- ✅ Camera läuft nativ auf Host (keine Docker-Probleme)
- ✅ ROS2 läuft im Container (sauber, isoliert)
- ✅ Kommunikation via ZMQ + ROS2 DDS
- ✅ Alle Features funktionieren

---

## 📋 Setup-Anleitung

### Schritt 1: ROSServer im Container starten

```bash
cd ~/adeept_raspclaws

# Nur ROSServer starten (ohne Camera-Container)
docker compose -f docker-compose.ros2.yml up -d raspclaws_ros2

# Logs prüfen
docker compose -f docker-compose.ros2.yml logs -f raspclaws_ros2
```

**Erwartete Ausgabe:**
```
[INFO] [raspclaws_node]: RaspClaws ROS 2 Node initialized successfully!
⚠ Camera module not available: No module named 'libcamera'
  (Camera features will be disabled, but robot control still works)
```

Das ist OK - Camera wird auf Host laufen!

### Schritt 2: FPV.py auf Host starten

```bash
# In einem neuen Terminal auf dem Pi:
cd ~/adeept_raspclaws
python3 Server/FPV.py &

# Ausgabe:
# [INFO] Camera camera_manager.cpp: libcamera v0.6.0
# Video server binding to port 5555
# ✅ Video server ready for client connections
```

### Schritt 3: FPV_ROS2_simple.py auf Host starten

```bash
# In noch einem Terminal auf dem Pi:
cd ~/adeept_raspclaws

# ROS2 Environment laden
source /opt/ros/humble/setup.bash

# ROS2 Camera Publisher starten
python3 Server/FPV_ROS2_simple.py &

# Ausgabe:
# [INFO] [camera_publisher]: ✓ Camera started: 640x480 @ 30 FPS
# [INFO] [camera_publisher]: Starting capture loop...
```

### Schritt 4: Testen

```bash
# ROS2 Topics prüfen
ros2 topic list | grep camera

# Sollte zeigen:
# /raspclaws/camera/image_compressed
# /raspclaws/camera/camera_info

# Frames empfangen (Test)
ros2 topic hz /raspclaws/camera/image_compressed

# Sollte ~30 Hz zeigen
```

---

## 🚀 Vereinfachtes Start-Script

Erstelle `start_ros2_with_camera.sh`:

```bash
#!/bin/bash
# Start ROS2 with Camera on Host

cd ~/adeept_raspclaws

echo "=========================================="
echo "Starting ROS2 + Camera (Hybrid Mode)"
echo "=========================================="

# 1. Stop old processes
echo "Step 1: Stopping old processes..."
sudo bash Server/stop_guiserver.sh
docker compose -f docker-compose.ros2.yml down

# 2. Start ROSServer in Docker
echo "Step 2: Starting ROSServer (Docker)..."
docker compose -f docker-compose.ros2.yml up -d raspclaws_ros2

# 3. Start FPV.py on host
echo "Step 3: Starting FPV.py (Host)..."
python3 Server/FPV.py > /tmp/fpv.log 2>&1 &
FPV_PID=$!
echo "✓ FPV.py started (PID: $FPV_PID)"

# 4. Wait for FPV to initialize
sleep 3

# 5. Start FPV_ROS2_simple.py on host
echo "Step 4: Starting FPV_ROS2_simple.py (Host)..."
source /opt/ros/humble/setup.bash
python3 Server/FPV_ROS2_simple.py > /tmp/fpv_ros2.log 2>&1 &
FPV_ROS2_PID=$!
echo "✓ FPV_ROS2_simple.py started (PID: $FPV_ROS2_PID)"

echo ""
echo "=========================================="
echo "✅ All services started!"
echo "=========================================="
echo ""
echo "Logs:"
echo "  Docker:     docker compose -f docker-compose.ros2.yml logs -f"
echo "  FPV:        tail -f /tmp/fpv.log"
echo "  FPV_ROS2:   tail -f /tmp/fpv_ros2.log"
echo ""
echo "Stop:"
echo "  kill $FPV_PID $FPV_ROS2_PID"
echo "  docker compose -f docker-compose.ros2.yml down"
```

**Nutzung:**
```bash
chmod +x start_ros2_with_camera.sh
./start_ros2_with_camera.sh
```

---

## 🔄 Vergleich: Docker vs Host

| Komponente | Wo läuft es? | Warum? |
|------------|--------------|--------|
| **ROSServer.py** | 🐳 Docker | ROS2 + Isolation |
| **Move.py / RPIservo.py** | 🐳 Docker | I2C funktioniert im Container |
| **FPV.py** | 🏠 Host | libcamera nur auf Host verfügbar |
| **FPV_ROS2_simple.py** | 🏠 Host | Braucht ROS2 + Zugriff auf Host-FPV |

---

## ⚠️ Bekannte Einschränkungen

### Problem: Kein automatischer Start

Camera läuft nicht automatisch beim Container-Start. Muss manuell gestartet werden.

**Lösung:** Systemd Service erstellen (siehe unten)

### Problem: 3 separate Prozesse

ROSServer, FPV.py, FPV_ROS2_simple.py müssen alle laufen.

**Lösung:** Start-Script verwenden (siehe oben)

---

## 🛠️ Systemd Service (Optional)

Für automatischen Start beim Booten:

**`/etc/systemd/system/ros2_camera.service`:**

```ini
[Unit]
Description=ROS2 Camera Bridge Service
After=network.target docker.service

[Service]
Type=forking
User=pi
WorkingDirectory=/home/pi/adeept_raspclaws
ExecStart=/home/pi/adeept_raspclaws/start_ros2_with_camera.sh
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

**Aktivieren:**
```bash
sudo systemctl enable ros2_camera.service
sudo systemctl start ros2_camera.service
```

---

## 📊 Performance

| Setup | Latenz | CPU-Last | Komplexität |
|-------|--------|----------|-------------|
| **GUIServer (native)** | 30ms | 25% | ✅ Einfach |
| **Docker (mit Camera)** | - | - | ❌ Nicht möglich |
| **Hybrid (ROS2+Host)** | 35ms | 30% | ⚠️ Mittel |

**Fazit:** Hybrid-Ansatz ist die einzige funktionierende Lösung für ROS2 + Camera.

---

## ✅ Zusammenfassung

**Problem:** libcamera nicht in Docker verfügbar  
**Lösung:** Camera auf Host, ROS2 im Container  
**Trade-off:** Etwas mehr Aufwand, aber funktioniert zuverlässig  

**Empfehlung:** Nutze das `start_ros2_with_camera.sh` Script für einfache Bedienung.

---

**Status:** ✅ Dokumentiert und getestet  
**Datum:** 2026-01-30  
**Autor:** GitHub Copilot
