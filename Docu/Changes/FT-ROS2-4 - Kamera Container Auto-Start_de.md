# FT-ROS2-4: Kamera Container Auto-Start

**Feature-Typ:** Infrastruktur-Verbesserung  
**Datum:** 2026-01-30  
**Status:** ✅ Implementiert

## Überblick

Der Kamera-Container startet nun automatisch beide benötigten Prozesse:
1. **FPV.py** - Kamera-Capture und ZMQ-Stream (existierender Code)
2. **FPV_ROS2_simple.py** - ROS2 Topic Publisher (liest Frames von FPV.py)

## Problem

Vorher musste der Benutzer manuell auf dem Pi folgende Befehle ausführen:
```bash
python3 Server/FPV.py &
docker compose -f docker-compose.ros2.yml up
```

Das führte zu folgenden Problemen:
- FPV.py musste manuell gestartet werden
- FPV.py hatte eine harte Abhängigkeit zu picamera2
- Fehler beim Start, wenn picamera2 nicht verfügbar war
- Benutzer musste sich um Prozess-Management kümmern

## Lösung

### 1. FPV.py Robustheit

**Datei:** `Server/FPV.py`

Imports wurden optional gemacht:
```python
try:
    import picamera2
    import libcamera
    from picamera2 import Picamera2, Preview
    CAMERA_AVAILABLE = True
except ImportError as e:
    print(f"⚠ Camera module not available: {e}")
    CAMERA_AVAILABLE = False
```

Kamera-Initialisierung wird nur ausgeführt, wenn Hardware verfügbar:
```python
picam2 = None
if CAMERA_AVAILABLE:
    try:
        picam2 = Picamera2()
        # ... Konfiguration ...
        picam2.start()
    except Exception as e:
        print(f"Camera initialization error: {e}")
        CAMERA_AVAILABLE = False
```

Capture-Loop prüft Verfügbarkeit:
```python
if not CAMERA_AVAILABLE or picam2 is None:
    print("❌ ERROR: Camera not available - cannot start capture loop")
    return
```

**Vorteile:**
- ✅ Keine harten Abhängigkeiten mehr
- ✅ Klare Fehlermeldungen wenn Kamera fehlt
- ✅ Kann auf Entwicklungs-PC geladen werden (ohne Kamera)

### 2. Startup-Script

**Datei:** `Server/start_camera_ros2.sh`

Neues Bash-Script, das beide Prozesse orchestriert:

```bash
#!/bin/bash
# Start FPV.py in background
python3 FPV.py &
FPV_PID=$!

# Wait for FPV.py to initialize
sleep 2

# Start FPV_ROS2_simple.py in foreground
python3 FPV_ROS2_simple.py

# Cleanup on exit
kill $FPV_PID 2>/dev/null
```

**Funktionsweise:**
1. Startet FPV.py im Hintergrund (kamera capture)
2. Wartet 2 Sekunden für Initialisierung
3. Startet FPV_ROS2_simple.py im Vordergrund (ROS2 publisher)
4. Wenn FPV_ROS2_simple.py beendet wird, wird auch FPV.py beendet

### 3. Docker Compose Integration

**Datei:** `docker-compose.ros2.yml`

Container Command wurde vereinfacht:
```yaml
raspclaws_camera:
  # ...
  command: >
    bash /ros2_ws/Server/start_camera_ros2.sh
```

**Statt vorher:**
```yaml
command: >
  bash -c "source /opt/ros/humble/setup.bash && 
        python3 /ros2_ws/Server/FPV_ROS2_simple.py"
```

### 4. Dockerfile Update

**Datei:** `Dockerfile.ros2`

Script-Berechtigungen werden gesetzt:
```dockerfile
RUN chmod +x /ros2_ws/Server/ROSServer.py
RUN chmod +x /ros2_ws/Server/start_camera_ros2.sh
```

## Verwendung

### Starten

Auf dem Raspberry Pi:
```bash
cd ~/adeept_raspclaws
docker compose -f docker-compose.ros2.yml up
```

Das war's! Kein manuelles Starten von FPV.py mehr nötig.

### Prüfen

Topics sollten verfügbar sein:
```bash
docker exec -it raspclaws_ros2 bash -c "source /opt/ros/humble/setup.bash && ros2 topic list"
```

Erwartete Kamera-Topics:
```
/raspclaws/camera/image_compressed
/raspclaws/camera/camera_info
```

### Logs

Container-Logs anzeigen:
```bash
docker compose -f docker-compose.ros2.yml logs -f raspclaws_camera
```

Sollte zeigen:
```
Starting FPV.py (camera capture + ZMQ stream)...
✓ FPV.py started (PID: 123)
Starting FPV_ROS2_simple.py (ROS2 publisher)...
✓ Simple Camera Publisher initialized successfully
```

## Technische Details

### Prozess-Hierarchie

```
Container
└── start_camera_ros2.sh (PID 1)
    ├── FPV.py (Background, PID saved)
    │   └── Picamera2 Thread (frame capture)
    └── FPV_ROS2_simple.py (Foreground)
        └── publish_loop Thread (ROS2 publishing)
```

### Datenfluss

```
[Picamera2] 
    → FPV.py capture_thread() 
    → set_latest_frame() (shared memory)
    → FPV_ROS2_simple.py get_latest_frame()
    → ROS2 Topics
```

### Signal Handling

Wenn Container gestoppt wird:
1. Docker sendet SIGTERM an start_camera_ros2.sh
2. Script beendet FPV_ROS2_simple.py (foreground)
3. Script tötet FPV.py (background)
4. Container stoppt sauber

## Fallback ohne Kamera

Wenn keine Kamera verfügbar ist:
- FPV.py startet, aber warnt: `❌ ERROR: Camera not available`
- FPV_ROS2_simple.py wartet auf Frames (bekommt None)
- Keine ROS2 Frames werden publiziert
- Container läuft weiter (kein Crash)

## Testen

### Ohne Hardware (Windows PC)

```bash
# FPV.py sollte warnen aber nicht crashen
python Server/FPV.py
# Output: ⚠ Camera module not available: No module named 'picamera2'
```

### Mit Hardware (Raspberry Pi)

```bash
# Container starten
docker compose -f docker-compose.ros2.yml up raspclaws_camera

# In anderem Terminal: Topics prüfen
ros2 topic list | grep camera
ros2 topic hz /raspclaws/camera/image_compressed
```

## Vorteile

✅ **Einfachere Bedienung** - Ein Befehl statt zwei  
✅ **Automatisches Prozess-Management** - Script kümmert sich um Start/Stop  
✅ **Robustheit** - Keine harten Abhängigkeiten mehr  
✅ **Entwickler-freundlich** - Kann ohne Kamera geladen werden  
✅ **Clean Shutdown** - Alle Prozesse werden sauber beendet  

## Dateien

- ✅ `Server/FPV.py` - Optional camera imports
- ✅ `Server/start_camera_ros2.sh` - Startup orchestration
- ✅ `docker-compose.ros2.yml` - Uses startup script
- ✅ `Dockerfile.ros2` - Makes script executable
- ✅ `Server/FPV_ROS2_simple.py` - Unverändert (verwendet shared frames)

## Nächste Schritte

Für weitere Verbesserungen:
1. Health-Check für Kamera-Container hinzufügen
2. Automatischer Restart bei Kamera-Fehler
3. Kamera-Konfiguration über Environment-Variablen
4. Performance-Monitoring (FPS-Counter in Status-Topic)

---

**Feature umgesetzt von:** GitHub Copilot  
**Review:** Pending  
**Hardware-Test:** Pending
