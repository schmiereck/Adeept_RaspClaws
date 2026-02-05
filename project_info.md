# Adeept_RaspClaws Project Information

## ⚠️ WICHTIG: Dokumentations-Richtlinien

**Diese Datei ist der zentrale Wissensstand für das gesamte Projekt.**

Bei jeder Arbeitssitzung:
1. **VOR dem Start:** Diese Datei lesen
2. **WÄHREND der Arbeit:** Erkenntnisse und Änderungen dokumentieren
3. **NACH wichtigen Schritten:** Status aktualisieren

---

## 🎯 Aktuelles Projekt: Hybrid-Architektur mit ROS2 (5. Februar 2026)

### Ziel
Roboter-Steuerung über ROS2 Humble (RoboStack) mit Kamera-Topics

### Architektur-Überblick

```
┌─────────────────────────────────────────────────────────┐
│  Raspberry Pi (IP: 192.168.2.126)                       │
│                                                          │
│  ┌────────────────────────────────────────────────────┐ │
│  │ GUIServer (system-Python 3.13)                     │ │
│  │ - Web GUI für manuellen Betrieb                    │ │
│  │ - Picamera2 (640x480 RGB888)                       │ │
│  │ - ZMQ Video Stream (Port 5555)                     │ │
│  │ - Servo Control über PCA9685                       │ │
│  │ - WS2812 LEDs                                      │ │
│  │ - TCP Command Server (Port 10223)                 │ │
│  └────────────────────────────────────────────────────┘ │
│                                                          │
│  ┌────────────────────────────────────────────────────┐ │
│  │ ROS2 Humble (RoboStack ros_env, Python 3.11)      │ │
│  │                                                     │ │
│  │ ┌─────────────────────────────────────────────┐   │ │
│  │ │ v4l2_camera Node                            │   │ │
│  │ │ - Liest /dev/video0 (libcamera v4l2)       │   │ │
│  │ │ - Publishes: sensor_msgs/Image             │   │ │
│  │ │ - Topic: /camera/image_raw                 │   │ │
│  │ └─────────────────────────────────────────────┘   │ │
│  │                                                     │ │
│  │ ┌─────────────────────────────────────────────┐   │ │
│  │ │ Robot Control Node (TODO)                  │   │ │
│  │ │ - Subscribes: cmd_vel                      │   │ │
│  │ │ - Controls: Servos, Movement               │   │ │
│  │ └─────────────────────────────────────────────┘   │ │
│  └────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────┘
```

### Warum Hybrid-Architektur?

**Problem:**
- libcamera/picamera2 sind für Python 3.13 kompiliert (system-wide)
- RoboStack ros_env verwendet Python 3.11
- Native C-Bibliotheken können nicht zwischen Python-Versionen geteilt werden

**Lösung:**
- **GUIServer**: System-Python 3.13 → picamera2 funktioniert
- **ROS2 Nodes**: RoboStack Python 3.11 → v4l2_camera über /dev/video0
- Beide greifen auf dieselbe Kamera zu (unterschiedliche Interfaces)

### Implementierungsplan

**Status: ✅ ERFOLGREICH IMPLEMENTIERT** (5. Februar 2026)

- [x] 1. .bashrc anpassen (ros_env nicht automatisch aktivieren)
- [x] 2. /dev/video0 Verfügbarkeit prüfen
- [x] 3. FPV_ROS2.py umbauen (Picamera2 → ZMQ-Bridge)
- [x] 4. ROSServer mit ZMQ-Bridge testen
- [x] 5. Alle ROS2-Topics verifizieren
- [ ] 6. Systemd Services für Auto-Start (optional, zukünftig)

### ✅ Test-Ergebnisse (5. Februar 2026)

**ROSServer erfolgreich getestet in RoboStack ros_env:**

```bash
# Alle verfügbaren ROS2 Topics:
/raspclaws/battery
/raspclaws/camera/camera_info          # ✓ ZMQ-Bridge funktioniert!
/raspclaws/camera/image_compressed     # ✓ ZMQ-Bridge funktioniert!
/raspclaws/camera/image_raw            # ✓ ZMQ-Bridge funktioniert!
/raspclaws/cmd_vel                     # Bewegungssteuerung
/raspclaws/cpu_temp
/raspclaws/cpu_usage
/raspclaws/gyro_data
/raspclaws/head_cmd                    # Kamera-Servo-Steuerung
/raspclaws/imu                         # MPU6050 Sensor
/raspclaws/ram_usage
/raspclaws/servo_positions
/raspclaws/status
```

**Erfolgreiche Komponenten:**
- ✅ GUIServer läuft in system-Python 3.13
- ✅ ROSServer läuft in RoboStack ros_env (Python 3.11)
- ✅ ZMQ-Bridge zwischen beiden funktioniert (tcp://127.0.0.1:5555)
- ✅ Kamera-Topics werden publiziert
- ✅ Lazy-Init funktioniert (Hardware wird bei erstem Command aktiviert)
- ✅ Servo-Control funktioniert
- ✅ LED-Control funktioniert

**Start-Befehle:**
```bash
# Terminal 1: GUIServer starten (system-Python 3.13)
ssh pi@192.168.2.126
cd /home/pi/Adeept_RaspClaws/Server
python3 GUIServer.py

# Terminal 2: ROSServer starten (RoboStack Python 3.11)
ssh pi@192.168.2.126
micromamba activate ros_env
cd /home/pi/Adeept_RaspClaws/Server
python3 ROSServer.py
```

---

## 🖥️ System-Konfiguration

**Hardware:**
- Raspberry Pi OS Lite arm64 (Debian Trixie)
- IP: 192.168.2.126
- Kamera: OV5647 (Pi Camera v1)

**Python-Umgebungen:**
1. **System-Python 3.13** (für GUIServer)
   - libcamera v0.6.0+rpt20251202
   - picamera2, opencv-python, numpy, zmq
   - adafruit-circuitpython-pca9685, adafruit-circuitpython-mpu6050

2. **RoboStack ros_env** (Python 3.11, für ROS2)
   - ROS2 Humble
   - Micromamba-basiert
   - v4l2_camera (zu installieren)

**Schnittstellen:**
- I2C: Aktiviert (PCA9685 auf 0x40)
- SPI: Aktiviert (WS2812 LEDs)
- GPIO: User `pi` in gpio-Gruppe

---

## ✅ GUIServer Status (Produktionsreif)

**Start-Befehl:**
```bash
ssh pi@192.168.2.126
cd /home/pi/Adeept_RaspClaws/Server
python3 GUIServer.py  # System-Python 3.13!
```

**Funktionen:**
✅ Video-Stream (640x480, Port 5555, ZMQ)
✅ Servo-Control (PCA9685)
✅ LED-Control (WS2812)
✅ Bewegung (forward, backward, left, right, Arc Left/Right)
✅ Kamera-Servo (lookLeft, lookRight, lookUp, lookDown)
✅ Power-Saving (Servos/Kamera starten AUS)

**Wichtige Erkenntnisse:**
- Kamera-Init in `capture_thread()` (nicht beim Import!) → verhindert Startup-Hang nach Reboot
- Power-Saving: `servo_standby_active = True`, `camera_paused_active = True` beim Start
- v4l2-Backend verfügbar über /dev/video0

---

## 🔧 Development Workflow

**Code-Änderungen auf Raspberry Pi:**
1. Lokal auf Windows bearbeiten
2. `git commit` und `git push`
3. Auf Raspberry Pi: `git pull`

**DO NOT:** Dateien direkt auf Pi ändern oder via scp übertragen!

---

## ⚠️ Bekannte Issues

**Nicht-kritisch (akzeptiert):**
- ADS7830 Batteriemonitor: Hardware nicht vorhanden
- LED breath_status_set: Attribut fehlt, aber LEDs funktionieren
- Debian Trixie libcamera Bug: Workaround implementiert (Kamera-Init in Thread)

**ROS2 Integration:**
- .bashrc aktiviert ros_env automatisch → verhindert GUIServer-Start
- **Lösung geplant:** .bashrc bereinigen, ros_env nur manuell aktivieren

---

**Letzte Aktualisierung**: 5. Februar 2026, 09:05 Uhr
**Status**: Hybrid-Architektur vollständig funktionsfähig ✅
- GUIServer produktionsreif (system-Python 3.13)
- ROSServer produktionsreif (RoboStack Python 3.11)
- ZMQ-Bridge erfolgreich implementiert und getestet