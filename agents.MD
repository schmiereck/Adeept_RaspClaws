# Agent Development Environment Documentation

## Projektübersicht

**Adeept_RaspClaws** - Roboter-Steuerungssoftware bestehend aus:
- **Client**: Windows GUI (Tkinter) zur Fernsteuerung
- **Server**: Raspberry Pi Server für Robotersteuerung und Video-Streaming

GitHub Repository: https://github.com/schmiereck/Adeept_RaspClaws.git

---

## 1. Entwicklungsumgebungen

### 1.1 Windows Entwicklungsrechner

**Betriebssystem**: Windows (PowerShell v5.1)

**IDE**: IntelliJ IDEA 2025.3

**Python Version**: Python 3.7.9 (64-bit)

**Projekt-Verzeichnis**:
```
C:\Users\SCMJ178\IdeaProjects\Adeept_RaspClaws\
```

**Virtuelles Environment**:
```
C:\Users\SCMJ178\IdeaProjects\Adeept_RaspClaws\.venv\
```

**Python SDK Path**:
```
C:\Users\SCMJ178\IdeaProjects\Adeept_RaspClaws\.venv\Scripts\python.exe
```

**Wichtige Python Packages**:
```
opencv-python==4.12.0.88
numpy==1.21.6
pyzmq
tkinter (standard library)
```

**Installation der Dependencies**:
```powershell
pip install -r requirements.txt
```

### 1.2 Raspberry Pi Server

**Betriebssystem**: Debian GNU/Linux (Raspberry Pi OS)
- Kernel: 6.12.47+rpt-rpi-v8 #1 SMP PREEMPT Debian 1:6.12.47-1+rpt1

**Python Version**: Python 3.13

**Hostname**: raspberrypi

**Standard User**: pi

**Projekt-Verzeichnis auf Pi**:
```
/home/pi/adeept_raspclaws/
```

**Altes Projekt-Verzeichnis (Backup)**:
```
/home/pi/adeept_raspclaws-original/
```

**Git Branch**: master

**Wichtige Hinweise**:
- ⚠️ Verzeichnisnamen sind nun **groß** geschrieben: `Server/` statt `server/`
- ⚠️ Hauptdatei heißt jetzt `GUIServer.py` (früher `server.py`)
- Link für Abwärtskompatibilität: `/home/pi/adeept_raspclaws/server/server.py` → `GUIServer.py`

**Hardware**:
- Kamera: OV5647 (libcamera v0.6.0+rpt20251202)
- PCA9685 PWM Controller: Adresse **0x40** (früher 0x5F)
- I²C Bus: busnum=1
- **ADS7830 ADC**: Adresse **0x48** (Batterie-Überwachung, 8-Kanal 8-Bit)
- WS2812 LEDs: 16 LEDs (über SPI, optional)
- MPU6050: Gyroskop/Accelerometer (optional)

**I²C Geräte Check**:
```bash
sudo i2cdetect -y 1
```
Erwartete Adressen:
- 0x40: PCA9685 PWM Controller
- 0x48: ADS7830 ADC (Batterie-Überwachung)
- 0x68: MPU6050 (optional)

---

## 2. Netzwerk-Konfiguration

### 2.1 Standard-Betrieb (Direktverbindung)

**Raspberry Pi IP**: 192.168.2.126 (oder variabel)

**Client IP**: 192.168.2.107 (oder variabel)

**Router Gateway**: 192.168.2.1

**WLAN**: speedport.ip

**Ports**:
- TCP 10223: Hauptverbindung (Client ↔ Server Kommandos)
- TCP 5555: Video-Stream (ZMQ PUB/SUB)
- Optional: TCP 8080 (Web-Interface)

**IP.txt Konfiguration** (Client):
```
192.168.2.126
```

### 2.2 SSH Tunnel (für Rechner mit Firewall-Einschränkungen)

**Problem**: Manche Entwicklungs-Rechner haben Firewall-Einschränkungen ohne Admin-Rechte.

**Lösung**: SSH Tunnel mit Port-Forwarding

**Tunnel-Befehl**:
```powershell
ssh -L 10223:localhost:10223 -L 5555:localhost:5555 pi@192.168.2.126
```

**IP.txt Konfiguration** (für SSH Tunnel):
```
127.0.0.1
```

**Automatisierung**: `Client/start_ssh_tunnel.bat` (interaktiv, Passwort erforderlich)

⚠️ **Wichtig**: SSH Tunnel muss manuell gestartet werden und läuft interaktiv (Passwort-Eingabe erforderlich). SSH-Keys können für passwortlose Authentifizierung konfiguriert werden (siehe `Docu/SSH_KEYS_SETUP_EN.md`).

**Fehlermeldung "channel 5: open failed"**:
- Diese Meldung kann auftreten, wenn der Server noch nicht bereit ist oder der Video-Stream noch nicht initialisiert wurde
- Normal bei Verbindungsaufbau, solange die GUI sich dann verbindet

---

## 3. Server-Betrieb auf Raspberry Pi

### 3.1 Systemd Service

**Service Name**: `robot_server.service`

**Service-Konfiguration**: `/etc/systemd/system/robot_server.service` (vermutlich)

**Befehle**:
```bash
# Service starten
sudo systemctl start robot_server.service

# Service stoppen
sudo systemctl stop robot_server.service

# Service neu starten
sudo systemctl restart robot_server.service

# Status prüfen
sudo systemctl status robot_server.service

# Logs anzeigen
sudo journalctl -u robot_server.service -n 100 --no-pager

# Logs live verfolgen
sudo journalctl -u robot_server.service -f
```

**Start-Kommando** (manuell):
```bash
sudo python3 /home/pi/adeept_raspclaws/server/server.py
```
(Der Link `server.py` zeigt auf `GUIServer.py`)

### 3.2 Projekt-Update vom Git

```bash
cd /home/pi/adeept_raspclaws
git pull
sudo systemctl restart robot_server.service
```

**Branch wechseln**:
```bash
git branch  # Aktuellen Branch anzeigen
git checkout master  # Auf master wechseln
```

---

## 4. Client-Betrieb (Windows GUI)

### 4.1 Start der GUI

**In IntelliJ**:
- Run Configuration: `GUI.py`
- Debug Mode: Zeigt Ausgaben
- Run Mode: Seit IntelliJ-Neustart funktioniert auch Run Mode

**Im Terminal**:
```powershell
cd C:\Users\SCMJ178\IdeaProjects\Adeept_RaspClaws\Client
..\..venv\Scripts\python.exe GUI.py
```

### 4.2 IP-Konfiguration

**Datei**: `Client/IP.txt`

Für **direkte Verbindung**:
```
192.168.2.126
```

Für **SSH Tunnel**:
```
127.0.0.1
```

**Wichtig**: Die GUI liest die IP aus dieser Datei beim Start.

### 4.3 Video-Stream (Footage-GUI)

Der Video-Stream wird als **separater Prozess** gestartet:
- Automatischer Start durch `GUI.py`
- Verwendet `Footage-GUI.py`
- Zeigt Kamera-Feed in separatem Fenster

**Hinweis**: Der Server sendet mehrfach `VIDEO_READY` Signale (10x über 10 Sekunden) um sicherzustellen, dass der Client bereit ist.

---

## 5. Architektur und Kommunikation

### 5.1 Kommunikationsfluss

```
┌─────────────────┐         TCP 10223          ┌─────────────────┐
│  Windows Client │ ◄────────────────────────► │  Raspberry Pi   │
│    (GUI.py)     │     Kommandos + Info       │ (GUIServer.py)  │
└─────────────────┘                            └─────────────────┘
         │                                              │
         │          ZMQ PUB/SUB (Port 5555)            │
         │          Video-Frames (JPEG)                 │
         ▼                                              ▼
┌─────────────────┐                            ┌─────────────────┐
│ Footage-GUI.py  │ ◄────────────────────────► │    FPV.py       │
│ (Video Window)  │                            │ (Video Capture) │
└─────────────────┘                            └─────────────────┘
```

### 5.2 Threading-Modell (Server)

**GUIServer.py**:
1. **Main Thread**: Socket-Verbindung und Kommando-Verarbeitung
2. **FPV Thread**: Video-Capture und Streaming (läuft kontinuierlich, auch ohne Client)
3. **Info Thread**: Sendet System-Informationen (CPU, RAM, Temp) an Client

**Wichtig**: FPV Thread wird **einmal beim Server-Start** gestartet und läuft permanent. Bei Client-Reconnect wird **kein** neuer Thread gestartet.

### 5.3 Reconnection-Handling

**Problem behoben**: Client kann sich mehrfach verbinden/trennen ohne Server-Neustart.

**Lösung**:
- FPV Thread läuft kontinuierlich (ZMQ PUB/SUB erlaubt mehrere Subscriber)
- Socket wird nach Disconnect korrekt geschlossen
- 2 Sekunden Wartezeit vor neuem Accept
- LED Status-Anzeige (blau = verbunden, atmend = wartend)

---

## 6. Features und Funktionen

### 6.1 Bewegungssteuerung

**Befehle**:
- `forward`, `backward`: Vorwärts/Rückwärts
- `left`, `right`: Links/Rechts drehen
- `DS`: Direction Stop (Stehen bleiben)
- `TS`: Turn Stop (Drehung stoppen)

**Geschwindigkeit**:
- `fast`: Schneller Modus
- `slow`: Langsamer Modus

**Smooth Mode**:
- `smooth`: Sanftes Anfahren/Stoppen für Bewegung
- `smoothOff`: Smooth Mode aus

Dokumentation: `Docu/SMOOTHMODE_ANALYSE.md`

### 6.2 Kamera-Steuerung

**Bewegung**:
- `up`, `down`: Kamera nach oben/unten
- `lookleft`, `lookright`: Kamera nach links/rechts
- `home`: Kamera in Neutral-Position

**Modi**:
- `steadyCamera`: Kamera-Stabilisierung an
- `steadyCameraOff`: Kamera-Stabilisierung aus
- `smoothCam`: Sanfte Kamera-Bewegungen
- `smoothCamOff`: Sanfte Kamera-Bewegungen aus

Dokumentation: `Docu/SMOOTHCAM_IMPLEMENTATION.md`

### 6.3 Computer Vision Features

- `findColor`: Farberkennung aktivieren
- `motionGet`: Bewegungserkennung (Watchdog)
- `CVFL`: Find Line Mode (Linienverfolgung)
- `stopCV`: Alle CV-Features stoppen

**Farbe setzen**:
```python
command = {'data': [r, g, b]}  # RGB Werte
```

### 6.4 LED-Steuerung (WS2812)

- `police`: Polizei-Licht Effekt
- `policeOff`: Zurück zu normalem Atem-Effekt

**Status-Farben**:
- Blau (64,128,255): Client verbunden
- Atmend (70,70,255): Wartet auf Client
- Grün: Accesspoint-Modus aktiv

⚠️ **Hinweis**: WS2812 LEDs sind optional. Server läuft auch ohne funktionierende LEDs im Mock-Mode.

### 6.5 System-Information

**Client empfängt alle 1-3 Sekunden**:
```
INFO:<CPU_TEMP> <CPU_USAGE> <RAM_USAGE> <BATTERY_VOLTAGE>
```

Beispiel: `INFO:58.0 4.3 42.6 7.8`
- CPU Temperatur: 58.0°C
- CPU Auslastung: 4.3%
- RAM Auslastung: 42.6%
- Batteriespannung: 7.8V

**GUI Anzeige**:
- CPU Temp, CPU Usage, RAM Usage: Immer sichtbar
- **Battery**: Mit Farb-Codierung (Grün/Orange/Rot) basierend auf Ladezustand
  - Grün: ≥ 60% (≥ 7.44V)
  - Orange: 30-60% (6.72V - 7.44V)
  - Rot: < 30% (< 6.72V)
  - Grau: "N/A" wenn Hardware nicht verfügbar

Dokumentation: `Docu/BATTERY_MONITOR_DE.md` / `Docu/BATTERY_MONITOR_EN.md`

---

## 7. Tastatur-Shortcuts (GUI)

Siehe Dokumentation:
- Deutsch: `Docu/KEYBOARD_SHORTCUTS.md`
- English: `Docu/KEYBOARD_SHORTCUTS_EN.md`

**Wichtigste Shortcuts**:
- `W/A/S/D`: Bewegung
- `Pfeiltasten`: Kamera-Steuerung
- `Q`: Programm beenden

---

## 8. Bekannte Probleme und Lösungen

### 8.1 PCA9685 Adresse geändert

**Problem**: Hardware verwendet Adresse 0x40, nicht 0x5F

**Lösung**: In `RPIservo.py` und `GUIServer.py` geändert

Dokumentation: `Docu/PCA9685_FIX_EN.md`

### 8.2 Reconnection Problem (GELÖST)

**Problem**: Video-Stream funktionierte nicht beim zweiten Connect

**Lösung**: FPV Thread läuft kontinuierlich, nicht pro Client

Dokumentation: `Docu/RECONNECTION_FIX.md`, `Docu/FPV_THREAD_FIX.md`

### 8.3 ZMQ Video Stream Binding

**Problem**: Video Stream musste für direkte Verbindung und SSH Tunnel funktionieren

**Lösung**: Server bindet auf `tcp://*:5555` (alle Interfaces), Client verbindet sich auf die IP aus IP.txt

Dokumentation: `Docu/ZMQ_PUBSUB_FIX.md`

### 8.4 CPU Last Optimierung

**Problem**: Server hatte 99% CPU Last

**Lösung**: Strategische `time.sleep(0.01)` in Haupt-Loops eingefügt

Dokumentation: `Docu/CPU_OPTIMIZATION.md`

### 8.5 LED Problem

**Problem**: WS2812 LEDs funktionierten nicht (SPI nicht verfügbar)

**Lösung**: Mock-Mode implementiert, Server läuft ohne LEDs

Dokumentation: `Docu/LED_PROBLEM_EN.md`

### 8.6 Connection Error Handling

**Problem**: Verschiedene Socket-Fehler bei Disconnect

**Lösung**: Umfassendes Exception Handling für alle Socket-Fehlertypen

Dokumentation: `Docu/CONNECTION_ERROR_HANDLING.md`

---

## 9. Change History

Alle Feature-Änderungen sind dokumentiert unter `Docu/Changes/`:

1. **FT1**: Connection Problem Fix
2. **FT2**: Video Stream Fix
3. **FT3**: Second Connect Fix
4. **FT4**: Second Connect Critical Bug Fix

Jede Änderung existiert in Deutsch (_de) und Englisch (_en).

---

## 10. Entwicklungs-Workflow

### 10.1 Typischer Workflow

1. **Änderungen auf Windows entwickeln**:
   ```powershell
   cd C:\Users\SCMJ178\IdeaProjects\Adeept_RaspClaws
   # Änderungen machen, testen mit lokaler GUI
   ```

2. **Committen und Pushen**:
   ```powershell
   git add .
   git commit -m "Beschreibung"
   git push origin master
   ```

3. **Auf Raspberry Pi aktualisieren**:
   ```bash
   ssh pi@192.168.2.126
   cd /home/pi/adeept_raspclaws
   git pull
   sudo systemctl restart robot_server.service
   ```

4. **Logs prüfen**:
   ```bash
   sudo journalctl -u robot_server.service -f
   ```

### 10.2 Testing

**Ohne Hardware** (Mock Mode):
- Server läuft auch ohne PCA9685, WS2812, Kamera
- Ideal für Logik-Tests

**Mit Hardware**:
- Vollständiger Test aller Features
- Kamera-Feed verifizieren
- Servo-Bewegungen testen

### 10.3 Debug-Tipps

**Client-Seite**:
- Debug Mode in IntelliJ zeigt alle Ausgaben
- `print()` Statements werden in Run-Fenster angezeigt

**Server-Seite**:
- `journalctl` für systemd Service Logs
- `sys.stdout.flush()` nach wichtigen Prints verwenden
- Fehler werden als Systemd-Logs gespeichert

**Netzwerk-Tests**:
```powershell
# Test ob Port offen
Test-NetConnection -ComputerName 192.168.2.126 -Port 10223

# Ping Test
ping 192.168.2.126

# Port-Listening prüfen (Windows)
netstat -ano | findstr :10223
```

---

## 11. Wichtige Dateien

### 11.1 Konfigurationsdateien

- `Client/IP.txt`: Server IP-Adresse
- `requirements.txt`: Python Dependencies
- `setup.py`: Package-Setup (falls vorhanden)

### 11.2 Hauptprogramme

**Client**:
- `Client/GUI.py`: Hauptprogramm mit Tkinter GUI
- `Client/Footage-GUI.py`: Video-Fenster (separater Prozess)
- `Client/ip_utils.py`: IP-Konfiguration Utilities

**Server**:
- `Server/GUIServer.py`: Hauptserver (ehemals server.py)
- `Server/FPV.py`: Video-Capture und Streaming
- `Server/Move.py`: Bewegungssteuerung
- `Server/RPIservo.py`: Servo-Controller (PCA9685)
- `Server/RobotLight.py`: LED-Steuerung (WS2812)
- `Server/Switch.py`: GPIO Switches
- `Server/Voltage.py`: Batterie-Überwachung

### 11.3 Dokumentation

Alle Dokumente unter `Docu/`:
- Englische Versionen enden mit `_EN.md`
- Deutsche Versionen ohne Suffix oder mit `_de.md`
- Change History unter `Docu/Changes/`

---

## 12. Zukünftige Entwicklung

### 12.1 ROS 2 Integration (Geplant)

**Vorbereitung**:
- Server-Architektur ist bereits modular
- Move.py kann als ROS2 Node adaptiert werden
- FPV Stream kann als ROS2 Topic publiziert werden

**Vorschlag**:
- Separate ROS2 Bridge Node erstellen
- Bestehende Socket-Kommunikation beibehalten (für Legacy-Clients)
- Parallele ROS2 Topics hinzufügen

### 12.2 Verbesserungsideen

- [ ] Web-basierte GUI (Flask/FastAPI)
- [ ] Gamepad/Controller Support
- [ ] Autonomous Navigation Modi
- [ ] Verbesserte Computer Vision (Object Detection)
- [ ] Video-Recording Funktion
- [ ] Telemetrie-Logging
- [ ] Multi-Client Support

---

## 13. Troubleshooting Checkliste

### Client startet nicht

- [ ] Virtuelles Environment aktiviert?
- [ ] OpenCV installiert? (`pip install opencv-python`)
- [ ] ZMQ installiert? (`pip install pyzmq`)
- [ ] IP.txt korrekt konfiguriert?

### Kein Video-Stream

- [ ] Server läuft? (`sudo systemctl status robot_server.service`)
- [ ] SSH Tunnel aktiv? (falls verwendet)
- [ ] Kamera angeschlossen? (Raspberry Pi)
- [ ] Port 5555 erreichbar?
- [ ] "VIDEO_READY" Signal im Log? (GUI Output)
- [ ] "channel 5: open failed" im SSH Tunnel → Normal, sollte verschwinden

### Steuerung funktioniert nicht

- [ ] TCP Port 10223 erreichbar?
- [ ] Server empfängt Kommandos? (journalctl Logs prüfen)
- [ ] PCA9685 erkannt? (i2cdetect -y 1)
- [ ] Bewegungs-Kommandos im Server-Log sichtbar?

### Zweiter Connect funktioniert nicht

- [ ] Aktuelle Version vom Git? (Reconnection Fix)
- [ ] Server-Logs prüfen: Zeigt "waiting for connection"?
- [ ] 2 Sekunden Wartezeit nach Disconnect?

### Hohe CPU Last

- [ ] Aktuelle Version? (CPU Optimization Fix)
- [ ] FPV Thread läuft nur einmal?
- [ ] Sleep Statements in Loops vorhanden?

---

## 14. Kontakte und Ressourcen

**Original Hersteller**: Adeept
- Website: www.adeept.com
- E-mail: support@adeept.com

**GitHub Repository**: https://github.com/schmiereck/Adeept_RaspClaws.git

**Entwickler**: schmiereck

**Original Autor** (Adeept Basis-Code): William (2018/08/22)

---

## 15. Versions-Historie

| Datum | Version | Änderung |
|-------|---------|----------|
| 2026-01-17 | 2.0 | Reconnection Fix, CPU Optimization, Smooth Cam |
| 2026-01-16 | 1.5 | PCA9685 Fix (0x40), SSH Tunnel Support |
| 2026-01-16 | 1.4 | ZMQ PUB/SUB Fix, Video Stream Stabilisierung |
| 2018-08-22 | 1.0 | Original Adeept Release |

---

**Letzte Aktualisierung**: 2026-01-18

**Status**: ✅ Produktiv - Client kann mehrfach verbinden, Video-Stream stabil
