# GUIServer Troubleshooting Guide

## Problem: "Address already in use"

### Symptome:

```
Error setting up connection: [Errno 98] Address already in use
zmq.error.ZMQError: Address already in use (addr='tcp://*:5555')
```

### Ursache:

Ein alter GUIServer-Prozess läuft noch und blockiert die Ports (10223 für TCP, 5555 für Video-Stream).

Dies kann passieren wenn:
- GUIServer mit Ctrl+C beendet wurde (nicht sauberes Shutdown)
- GUIServer gecrasht ist
- Mehrere GUIServer-Instanzen gestartet wurden

### Lösung:

#### Option 1: Cleanup-Script verwenden (empfohlen)

```bash
cd ~/adeept_raspclaws
bash Server/stop_guiserver.sh
```

Dann GUIServer neu starten:
```bash
sudo python3 Server/GUIServer.py
```

#### Option 2: Manuelle Bereinigung

1. Finde alle laufenden GUIServer-Prozesse:
```bash
ps aux | grep GUIServer.py | grep -v grep
```

2. Töte die Prozesse (ersetze <PID> mit der tatsächlichen Process-ID):
```bash
sudo kill -9 <PID>
```

3. Prüfe, welche Prozesse die Ports blockieren:
```bash
sudo lsof -i :10223  # TCP Server-Port
sudo lsof -i :5555   # Video-Stream-Port
```

4. Töte die Prozesse, die die Ports blockieren:
```bash
sudo kill -9 <PID>
```

### Prävention:

**Sauberes Beenden:**
Beende GUIServer immer sauber mit Ctrl+C und warte auf das vollständige Shutdown.

**Vor Neustart prüfen:**
```bash
# Prüfe, ob noch Prozesse laufen
ps aux | grep GUIServer.py | grep -v grep

# Wenn ja, verwende cleanup script
bash Server/stop_guiserver.sh
```

## Docker vs GUIServer

⚠️ **Wichtig:** Docker-Container und GUIServer können nicht gleichzeitig laufen!

Beide verwenden:
- GPIO-Pins (für Servos)
- Kamera-Hardware
- Teilweise die gleichen Ports

### Verwendung:

**Option 1: GUIServer (normaler Betrieb)**
```bash
# Docker stoppen
docker compose -f docker-compose.ros2.yml down

# Alte Prozesse bereinigen
bash Server/stop_guiserver.sh

# GUIServer starten
sudo python3 Server/GUIServer.py
```

**Option 2: ROS2 Server (Container-Betrieb)**
```bash
# GUIServer beenden (Ctrl+C oder stop_guiserver.sh)
bash Server/stop_guiserver.sh

# Docker starten
docker compose -f docker-compose.ros2.yml up
```

## Weitere Probleme

### GPIO busy

**Symptom:**
```
lgpio.error: 'GPIO busy'
```

**Ursache:** Docker-Container oder anderer Prozess verwendet GPIO.

**Lösung:**
```bash
docker compose -f docker-compose.ros2.yml down
bash Server/stop_guiserver.sh
```

### Camera in use

**Symptom:**
```
Pipeline handler in use by another process
```

**Ursache:** Docker-Container oder anderer Prozess verwendet die Kamera.

**Lösung:**
```bash
docker compose -f docker-compose.ros2.yml down
# Prüfe auf laufende Kamera-Prozesse
ps aux | grep -i 'fpv\|camera\|picamera' | grep -v grep
```

## Logs

GUIServer-Logs anzeigen (wenn als Service läuft):
```bash
sudo journalctl -u gui_server.service -f
```

Prozess-Status prüfen:
```bash
sudo systemctl status gui_server.service
```

## Support

Bei weiteren Problemen:
1. Logs sammeln
2. Prozessliste sammeln: `ps aux | grep python`
3. Port-Status sammeln: `sudo netstat -tulpn | grep -E '5555|10223'`
