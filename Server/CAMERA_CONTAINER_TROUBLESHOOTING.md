em GUIServer bewiesen. Der FPV-Stream wurde in der GUI angezeigt. d# Camera Container Troubleshooting

## Problem: Kamera-Container startet in Endlosschleife

### Symptome:

```
raspclaws_camera  | ❌ ERROR: Camera not available - cannot start capture loop
raspclaws_camera  |    FPV.py needs camera hardware to work
raspclaws_camera  | Video server binding to port 5555 (PUB socket, allows multiple clients)
# ... wiederholt sich mehrfach
```

### Ursache:

Die Kamera ist nicht verfügbar oder nicht korrekt angeschlossen.

### Lösung:

#### Option 1: Kamera-Container deaktivieren (wenn keine Kamera vorhanden)

Der ROSServer funktioniert auch **ohne Kamera-Container**. Deaktiviere ihn einfach:

```bash
cd ~/adeept_raspclaws

# docker-compose.ros2.yml bearbeiten
nano docker-compose.ros2.yml

# Kommentiere den gesamten raspclaws_camera Block aus:
#  raspclaws_camera:
#    image: adeept_raspclaws-raspclaws_ros2:latest
#    ...
```

**Oder verwende Profile (empfohlen):**

```bash
# Nur ROSServer starten (ohne Kamera)
docker compose -f docker-compose.ros2.yml up raspclaws_ros2

# Mit Kamera starten
docker compose -f docker-compose.ros2.yml up
```

#### Option 2: Kamera anschließen und aktivieren

1. **Kamera anschließen:**
   - Raspberry Pi herunterfahren
   - Kamera-Ribbon-Kabel an CSI-Port anschließen
   - Raspberry Pi hochfahren

2. **Kamera aktivieren:**
   ```bash
   sudo raspi-config
   # Navigate to: Interface Options → Camera → Enable
   # Reboot
   ```

3. **Kamera testen:**
   ```bash
   # libcamera Test
   libcamera-hello

   # Überprüfe /dev/video0
   ls -l /dev/video0
   # Sollte existieren: crw-rw---- 1 root video ...
   ```

4. **Container neu starten:**
   ```bash
   docker compose -f docker-compose.ros2.yml down
   docker compose -f docker-compose.ros2.yml up
   ```

#### Option 3: Mit Mock-Kamera testen (ohne Hardware)

Wenn du nur die ROS2-Integration testen willst, ohne echte Kamera:

```bash
# Container deaktivieren
docker compose -f docker-compose.ros2.yml up raspclaws_ros2

# Keine Kamera-Topics, aber Bewegung funktioniert:
ros2 topic pub --once /raspclaws/cmd_vel geometry_msgs/Twist "{linear: {z: 0.5}}"
```

## Verbesserungen ab Version 2026-01-30

### 1. Script verhindert Restart-Loop

**Datei:** `Server/start_camera_ros2.sh`

Das Script prüft jetzt vor dem Start, ob die Kamera verfügbar ist:

```bash
# Check if camera device exists
if [ ! -e /dev/video0 ]; then
    echo "❌ ERROR: Camera device /dev/video0 not found"
    echo "   Container will sleep to prevent restart loop"
    sleep infinity  # ✅ Verhindert Restart-Loop
    exit 1
fi
```

**Vorher:** Container crashte sofort → Docker startete neu → Endlosschleife  
**Nachher:** Container schläft → Docker startet NICHT neu → Keine Endlosschleife

### 2. Bessere Restart-Policy

**Datei:** `docker-compose.ros2.yml`

```yaml
raspclaws_camera:
  # ...
  restart: on-failure  # ✅ Nur bei Crash, nicht bei sleep
  healthcheck:
    test: ["CMD-SHELL", "pgrep -f 'FPV.py' || exit 1"]
    interval: 10s
    timeout: 5s
    retries: 3
```

**Vorher:** `restart: unless-stopped` → Startete auch bei Exit 1  
**Nachher:** `restart: on-failure` → Startet nur bei echtem Crash

### 3. Health-Check

Docker prüft jetzt regelmäßig, ob FPV.py läuft:

```bash
# Container-Status prüfen
docker compose -f docker-compose.ros2.yml ps

# Zeigt:
# NAME                STATUS                          HEALTH
# raspclaws_ros2     Up 5 minutes                    healthy
# raspclaws_camera   Up 5 minutes                    unhealthy (wenn keine Kamera)
```

## Verwendung

### Szenario 1: Mit Kamera

```bash
# Kamera angeschlossen und aktiviert
docker compose -f docker-compose.ros2.yml up

# Erwartete Ausgabe:
# raspclaws_camera  | ✓ FPV.py started successfully (PID: 123)
# raspclaws_camera  | Starting FPV_ROS2_simple.py (ROS2 publisher)...
```

### Szenario 2: Ohne Kamera (Container deaktiviert)

```bash
# Nur ROSServer starten
docker compose -f docker-compose.ros2.yml up raspclaws_ros2

# raspclaws_camera wird NICHT gestartet
```

### Szenario 3: Ohne Kamera (Container schläft)

```bash
# Beide Container starten, aber Kamera-Container schläft
docker compose -f docker-compose.ros2.yml up

# Erwartete Ausgabe:
# raspclaws_camera  | ❌ ERROR: Camera device /dev/video0 not found
# raspclaws_camera  |    Container will sleep to prevent restart loop
# (Container läuft weiter, aber tut nichts)
```

## Logs

### Kamera-Container Logs anzeigen

```bash
# Live-Logs
docker compose -f docker-compose.ros2.yml logs -f raspclaws_camera

# Letzte 50 Zeilen
docker compose -f docker-compose.ros2.yml logs --tail=50 raspclaws_camera
```

### Häufige Fehlermeldungen

#### 1. "Camera device /dev/video0 not found"

**Ursache:** Kamera nicht angeschlossen oder nicht aktiviert

**Lösung:** Siehe Option 2 oben (Kamera anschließen und aktivieren)

#### 2. "Pipeline handler in use by another process"

**Ursache:** Anderer Prozess (z.B. GUIServer) verwendet die Kamera

**Lösung:**
```bash
# GUIServer stoppen
bash Server/stop_guiserver.sh

# Container neu starten
docker compose -f docker-compose.ros2.yml restart raspclaws_camera
```

#### 3. "FPV.py failed to start or crashed"

**Ursache:** picamera2 Fehler, meist Kamera-Hardware-Problem

**Lösung:**
```bash
# Kamera testen
libcamera-hello

# Wenn das nicht funktioniert:
sudo raspi-config
# Interface Options → Camera → Disable → Enable
# Reboot
```

## Best Practices

### Entwicklung ohne Kamera

Wenn du an der ROS2-Integration arbeitest, aber keine Kamera hast:

```bash
# docker-compose.ros2.yml
# Kommentiere raspclaws_camera Block komplett aus

# ODER starte nur ROSServer:
docker compose -f docker-compose.ros2.yml up raspclaws_ros2
```

**Vorteil:** Schnellerer Start, weniger Fehler-Logs

### Production mit Kamera

Stelle sicher, dass die Kamera funktioniert, bevor du den Container startest:

```bash
# 1. Kamera testen
libcamera-hello

# 2. Device prüfen
ls -l /dev/video0

# 3. Container starten
docker compose -f docker-compose.ros2.yml up
```

## Weitere Informationen

- **FPV.py Dokumentation:** `Server/FPV.py` (Kommentare im Code)
- **ROS2 Integration:** `Docu/Changes/FT-ROS2-1 - ROS2 Integration_de.md`
- **Kamera-Container:** `Docu/Changes/FT-ROS2-4 - Kamera Container Auto-Start_de.md`

---

**Erstellt:** 2026-01-30  
**Autor:** GitHub Copilot  
**Problem behoben:** Endlos-Restart-Loop bei fehlender Kamera
