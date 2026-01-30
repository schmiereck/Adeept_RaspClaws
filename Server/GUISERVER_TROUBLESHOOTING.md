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

⚠️ **WICHTIG:** Verwende **Ctrl+C** zum Beenden, **NICHT Ctrl+Z**!

- **Ctrl+C** (✅ richtig): Beendet den Prozess sauber (Signal Handler schließt Sockets)
- **Ctrl+Z** (❌ falsch): Pausiert den Prozess nur - läuft im Hintergrund weiter und blockiert Ports!

Wenn du versehentlich Ctrl+Z gedrückt hast:
```bash
# Liste pausierte Jobs
jobs

# Prozess im Vordergrund fortsetzen und dann mit Ctrl+C beenden
fg

# ODER direkt töten
bash Server/stop_guiserver.sh
```

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

---

## ⚠️ WICHTIG: Ctrl+Z vs Ctrl+C

**Verwende IMMER Ctrl+C zum Beenden, NIEMALS Ctrl+Z!**

| Tastenkombination | Effekt | Sockets/Ports | Richtig? |
|-------------------|--------|---------------|----------|
| **Ctrl+C** | Beendet Prozess sauber | ✅ Werden freigegeben | ✅ **JA** |
| **Ctrl+Z** | Pausiert Prozess nur | ❌ Bleiben blockiert | ❌ **NEIN** |

**Wenn du versehentlich Ctrl+Z gedrückt hast:**
```bash
# Job-Liste anzeigen
jobs

# Prozess zurückholen und mit Ctrl+C beenden
fg

# ODER direkt alle Prozesse töten
bash Server/stop_guiserver.sh
```

Der GUIServer hat jetzt einen Signal Handler, der bei Ctrl+C:
- ✅ Sockets sauber schließt
- ✅ Video-Thread stoppt
- ✅ LEDs ausschaltet
- ✅ Move-Module aufräumt
- ✅ Ports freigibt

**Vermeide "Stopped" Prozesse:**
```bash
# Prüfe auf gestoppte Prozesse
jobs

# Oder systemweit
ps aux | grep GUIServer
# Wenn "T" in der Status-Spalte steht = Stopped (schlecht!)
```

