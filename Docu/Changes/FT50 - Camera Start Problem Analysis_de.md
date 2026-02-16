# FT50 - Analyse: Kamera-Stream Problem beim Service-Start

**Datum:** 2026-02-16
**Typ:** Analyse / Problem Investigation
**Priorität:** Mittel
**Komponente:** Server/FPV.py, Server/GUIServer.py, systemd Services

## Problem

Der GUIServer (als systemd Service gestartet) liefert beim ersten Start manchmal **keinen Kamera-Stream**. Erst nach einem Service-Restart (`sudo systemctl restart gui_server.service`) funktioniert der Stream.

**Vermutung:** Möglicher Zusammenhang mit der Startreihenfolge des ROSServer.

## Systemarchitektur

### Zwei unabhängige Services

```
┌─────────────────────────────────────────────────────────┐
│  Raspberry Pi (Systemd Services)                        │
│                                                          │
│  ┌────────────────────────────────────────────────────┐ │
│  │ gui_server.service                                 │ │
│  │ - User: pi                                         │ │
│  │ - After: network.target                           │ │
│  │ - ExecStart: /usr/bin/python3 ./Server/GUIServer.py │
│  │ - Restart: always (RestartSec=5)                  │ │
│  │                                                     │ │
│  │ → Startet GUIServer.py (system Python 3.13)       │ │
│  │ → Initialisiert FPV Thread (Kamera + ZMQ)         │ │
│  └────────────────────────────────────────────────────┘ │
│                                                          │
│  ┌────────────────────────────────────────────────────┐ │
│  │ ros_server.service                                 │ │
│  │ - User: pi                                         │ │
│  │ - After: network.target                           │ │
│  │ - ExecStart: /bin/bash run_rosserver_systemd.sh   │ │
│  │ - Restart: always (RestartSec=5)                  │ │
│  │                                                     │ │
│  │ → Startet ROSServer.py (micromamba ros_env)       │ │
│  │ → Benötigt KEINE Kamera (v4l2 oder ZMQ-Bridge)    │ │
│  └────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────┘
```

### Wichtige Erkenntnis: Keine Service-Abhängigkeiten

**Beide Services haben KEINE gegenseitigen Abhängigkeiten:**
- Kein `Requires=`, `Wants=`, `Before=`, `After=` Bezug aufeinander
- Beide starten nur nach `network.target`
- Startreihenfolge ist **nicht deterministisch**

### Kamera-Zugriff

**GUIServer:**
- Nutzt `picamera2` (system Python 3.13)
- Direkter Zugriff auf Kamera über libcamera
- Erstellt FPV Thread beim Server-Start

**ROSServer:**
- Nutzt `v4l2_camera` (ROS2 node in ros_env)
- Zugriff über `/dev/video0` (libcamera-v4l2)
- Alternativ: ZMQ-Bridge von GUIServer (optional)

**→ KEIN gemeinsamer Hardware-Zugriff zwischen den Services!**

## Kamera-Initialisierung im GUIServer

### Ablauf beim Server-Start

```python
# GUIServer.py - main() Funktion:

1. move.init_all()                    # Servo-Initialisierung
2. ws2812 = initialize_leds()         # LED-Initialisierung
3. start_video_thread()               # ← FPV Thread Start
4. move.standby()                     # Servos in Standby
5. FPV.pause_stream()                 # Kamera initial PAUSIERT
6. while 1:                           # Client-Connect-Loop
       tcpSerSock.accept()
       run()
```

### FPV Thread Initialisierung

```python
# Server/FPV.py - capture_thread() Methode:

def capture_thread(self, IPinver):
    # 1. ZMQ Socket Setup
    footage_socket = context.socket(zmq.PUB)
    footage_socket.bind('tcp://*:5555')
    
    # 2. Video Ready Marker
    with open('/tmp/video_ready', 'w') as f:
        f.write('1')
    
    # 3. Kamera-Initialisierung (MIT 5-SEKUNDEN TIMEOUT!)
    picam2 = Picamera2()
    preview_config = picam2.create_preview_configuration(
        main={"size": (320, 240), "format": "RGB888"},
        buffer_count=4,
        queue=True
    )
    picam2.configure(preview_config)
    
    # ⚠️ KRITISCHER PUNKT: picam2.start() mit Timeout
    def start_camera():
        picam2.start()
    
    start_thread = threading.Thread(target=start_camera, daemon=True)
    start_thread.start()
    start_thread.join(timeout=5.0)  # ← 5s Timeout!
    
    if start_thread.is_alive():
        print("[FPV] ❌ TIMEOUT: Camera start hung (Debian Trixie bug)")
        return  # ← Thread beendet sich!
    
    # 4. Video Capture Loop (nur wenn Kamera erfolgreich gestartet)
    while True:
        frame_image = picam2.capture_array()
        if not camera_paused:
            footage_socket.send(...)
```

## Bekanntes Problem: Debian Trixie Kamera-Timeout

### Dokumentation in FPV.py (Zeile 197-246)

```python
# Initialize camera with timeout workaround for Debian Trixie bug
# 
# Known Issue: picam2.start() kann nach Reboot hängenbleiben
# Symptom: Blockiert für >5 Sekunden
# Lösung: Timeout-Thread mit 5s Deadline
```

### Was passiert beim Timeout?

1. `picam2.start()` blockiert länger als 5 Sekunden
2. Timeout-Thread erkennt das Hängen
3. FPV Thread gibt Fehlermeldung aus:
   ```
   [FPV.capture_thread] ❌ TIMEOUT: Camera start hung (Debian Trixie bug)
   [FPV.capture_thread]    picam2.start() blocked for >5 seconds
   [FPV.capture_thread]    This is a known Debian Trixie issue after reboot
   ```
4. **FPV Thread beendet sich komplett** (return)
5. ZMQ Socket ist zwar gebunden, aber keine Frames werden gesendet
6. Client verbindet zu ZMQ, aber erhält keine Frames

### Nach Service-Restart funktioniert es

**Warum?**
- Kamera wurde durch ersten Versuch "aufgeweckt"
- libcamera Hardware-State ist jetzt initialisiert
- Zweiter Start funktioniert ohne Timeout

## Mögliche Ursachen

### 1. Kamera Hardware nicht bereit beim Boot

**Timing-Problem:**
```
Zeit:     System                      GUIServer
--------------------------------------------------------------
t=0.0     Boot startet                -
t=5.0     network.target erreicht     -
t=5.1     gui_server.service startet  GUIServer.py main()
t=5.2     ...                         FPV Thread startet
t=5.3     ...                         picam2 = Picamera2()
t=5.4     ...                         picam2.configure(...)
t=5.5     ...                         picam2.start() → HÄNGT!
          
          ⚠️ Kamera Hardware noch nicht vollständig initialisiert
          ⚠️ libcamera Treiber braucht noch Zeit
          
t=10.5    ...                         Timeout! FPV Thread beendet sich
t=15.0    Service Restart             -
t=15.1    ...                         FPV Thread startet (2. Versuch)
t=15.2    ...                         picam2.start() → FUNKTIONIERT! ✅
```

**Hypothese:** Kamera Hardware/Treiber braucht länger als `network.target` + 0.1s zum Initialisieren.

### 2. Ressourcen-Konflikt beim Parallel-Start

**Wenn beide Services gleichzeitig starten:**
```
Zeit:     gui_server.service          ros_server.service
--------------------------------------------------------------
t=0.0     Startet                     Startet
t=0.1     FPV Thread startet          ROS2 node startet
t=0.2     picam2.start()              v4l2_camera startet (?)
          
          ⚠️ Möglicher Konflikt auf Hardware-Ebene?
          ⚠️ Beide greifen auf libcamera zu?
```

**Aber:** ROSServer nutzt v4l2 über `/dev/video0`, nicht direkt libcamera.

**Wahrscheinlichkeit:** Eher gering, da v4l2 und libcamera getrennte Interfaces sind.

### 3. Power Management / USB Power

**Hypothese:** Kamera bekommt beim Boot nicht sofort genug Power.
- USB Kamera braucht Zeit zum Hochfahren
- Power Management könnte Kamera verzögern

**Aber:** OV5647 ist CSI-Kamera (kein USB), also weniger wahrscheinlich.

### 4. systemd Service-Zeitpunkt

**Problem:** `network.target` bedeutet nur "Netzwerk-Stack verfügbar", nicht "alle Hardware bereit".

**Mögliche Lösung:** Service verzögern, bis Hardware garantiert bereit ist.

## Diagnose-Schritte

### 1. Logs analysieren

```bash
# Beim ersten Start (fehlgeschlagen):
sudo journalctl -u gui_server.service -n 100 --no-pager

# Kamera-Timeout-Meldung suchen:
# "[FPV.capture_thread] ❌ TIMEOUT: Camera start hung"
```

### 2. Timing messen

```bash
# Wie lange braucht Kamera zum Initialisieren nach Reboot?
sudo reboot

# Nach Reboot:
time python3 -c "from picamera2 import Picamera2; p = Picamera2(); p.start()"
# → Misst Zeit bis Kamera bereit ist
```

### 3. libcamera Status prüfen

```bash
# Ist libcamera Hardware bereit?
libcamera-hello --list-cameras

# Welche /dev/video* Devices existieren?
ls -la /dev/video*
```

### 4. Service-Startreihenfolge beobachten

```bash
# Welcher Service startet zuerst?
sudo systemctl show gui_server.service | grep ExecMainStartTimestamp
sudo systemctl show ros_server.service | grep ExecMainStartTimestamp

# Beide Services restarten und Timing beobachten:
sudo systemctl restart gui_server.service ros_server.service
sudo journalctl -f -u gui_server.service -u ros_server.service
```

## Lösungsansätze

### Option 1: Service Dependency hinzufügen

**Ziel:** Stelle sicher, dass Kamera-Hardware bereit ist, bevor Service startet.

```ini
# systemd/gui_server.service

[Unit]
Description=ROS2 Adeept RaspClaws GUI Server
After=network.target systemd-udev-settle.service
Wants=systemd-udev-settle.service

[Service]
...
```

**`systemd-udev-settle.service`:** Wartet, bis alle udev-Geräte initialisiert sind.

**⚠️ Vorsicht:** Kann Boot verlangsamen (wartet auf alle Hardware-Geräte).

### Option 2: ExecStartPre mit Delay

**Ziel:** Warte explizit N Sekunden vor dem Start.

```ini
[Unit]
Description=ROS2 Adeept RaspClaws GUI Server
After=network.target

[Service]
ExecStartPre=/bin/sleep 5
ExecStart=/usr/bin/python3 ./Server/GUIServer.py
...
```

**Vorteil:** Einfach, gibt Kamera Zeit zum Initialisieren.
**Nachteil:** Feste Verzögerung, nicht elegant.

### Option 3: FPV Thread mit Retry-Logik

**Ziel:** Wenn Kamera-Start fehlschlägt, automatisch nach N Sekunden wiederholen.

```python
# Server/FPV.py - capture_thread() Modifikation:

def capture_thread(self, IPinver):
    MAX_RETRIES = 3
    RETRY_DELAY = 10  # Sekunden
    
    for retry in range(MAX_RETRIES):
        try:
            # Kamera-Initialisierung versuchen
            picam2 = Picamera2()
            ...
            start_thread.join(timeout=5.0)
            
            if start_thread.is_alive():
                raise Exception("Camera start timeout")
            
            # Erfolg! → Weiter zur Capture Loop
            break
            
        except Exception as e:
            if retry < MAX_RETRIES - 1:
                print(f"[FPV] Camera init failed (retry {retry+1}/{MAX_RETRIES})")
                print(f"[FPV] Waiting {RETRY_DELAY}s before retry...")
                time.sleep(RETRY_DELAY)
            else:
                print(f"[FPV] Camera init failed after {MAX_RETRIES} retries")
                return
    
    # Capture Loop (nur wenn erfolgreich)
    while True:
        ...
```

**Vorteil:** Automatische Wiederherstellung ohne manuellen Restart.
**Nachteil:** Verzögert Video-Start um 10-20s, wenn Problem auftritt.

### Option 4: libcamera Vorinitialisierung

**Ziel:** Initialisiere Kamera einmal vor dem Daemon-Start.

```bash
# Neues Script: Server/init_camera.sh
#!/bin/bash
# Trigger camera initialization
libcamera-hello --timeout 1000  # 1 Sekunde Kamera aktivieren
echo "Camera initialized"
```

```ini
# systemd/gui_server.service
[Service]
ExecStartPre=/home/pi/Adeept_RaspClaws/Server/init_camera.sh
ExecStart=/usr/bin/python3 ./Server/GUIServer.py
```

**Vorteil:** Kamera ist garantiert bereit, wenn GUIServer startet.
**Nachteil:** Extra Prozess, könnte Boot um 1-2s verzögern.

### Option 5: Service Order mit Before/After

**Ziel:** Verhindere parallelen Start von gui_server und ros_server.

```ini
# systemd/gui_server.service
[Unit]
After=network.target
Before=ros_server.service

# systemd/ros_server.service
[Unit]
After=network.target gui_server.service
```

**Vorteil:** Deterministische Reihenfolge.
**Nachteil:** ROSServer startet erst nach GUIServer (verzögert).

**⚠️ Aber:** ROSServer nutzt v4l2, nicht picamera2 → wahrscheinlich kein Konflikt.

## Empfohlene Lösung

### Kombination: Option 3 + Option 2

**1. FPV Thread mit Retry-Logik (Option 3):**
- Automatische Wiederherstellung bei Timeout
- Keine manuelle Intervention nötig

**2. Kleiner ExecStartPre Delay (Option 2):**
- 3-5 Sekunden Delay vor GUIServer-Start
- Gibt Kamera Zeit, ohne lange zu warten

```ini
# systemd/gui_server.service

[Unit]
Description=ROS2 Adeept RaspClaws GUI Server
After=network.target

[Service]
User=pi
WorkingDirectory=/home/pi/Adeept_RaspClaws/
ExecStartPre=/bin/sleep 3
ExecStart=/usr/bin/python3 ./Server/GUIServer.py
Restart=always
RestartSec=5
StandardOutput=journal
StandardError=journal
SyslogIdentifier=gui_server

[Install]
WantedBy=multi-user.target
```

```python
# Server/FPV.py - capture_thread() mit Retry:

def capture_thread(self, IPinver):
    print(f"[FPV.capture_thread] Starting capture thread for IP {IPinver}...")
    
    # ... ZMQ Setup (bleibt gleich) ...
    
    # Initialize camera with retry logic
    MAX_RETRIES = 3
    RETRY_DELAY = 10
    camera_initialized = False
    
    for retry in range(MAX_RETRIES):
        try:
            print(f"[FPV.capture_thread] Camera init attempt {retry+1}/{MAX_RETRIES}")
            picam2 = Picamera2()
            preview_config = picam2.create_preview_configuration(...)
            picam2.configure(preview_config)
            
            # Start with timeout
            start_success = [False]
            def start_camera():
                picam2.start()
                start_success[0] = True
            
            start_thread = threading.Thread(target=start_camera, daemon=True)
            start_thread.start()
            start_thread.join(timeout=5.0)
            
            if start_thread.is_alive():
                print(f"[FPV] ⚠️ Camera start timeout (attempt {retry+1})")
                if retry < MAX_RETRIES - 1:
                    print(f"[FPV] Retrying in {RETRY_DELAY}s...")
                    time.sleep(RETRY_DELAY)
                    continue
                else:
                    print("[FPV] ❌ Camera init failed after all retries")
                    return
            elif start_success[0]:
                print("[FPV] ✓ Camera started successfully")
                camera_initialized = True
                break
                
        except Exception as e:
            print(f"[FPV] ⚠️ Camera init error: {e}")
            if retry < MAX_RETRIES - 1:
                print(f"[FPV] Retrying in {RETRY_DELAY}s...")
                time.sleep(RETRY_DELAY)
            else:
                print("[FPV] ❌ Camera init failed after all retries")
                return
    
    if not camera_initialized:
        print("[FPV] ❌ Camera not available - FPV thread terminating")
        return
    
    # Capture loop (nur wenn Kamera erfolgreich)
    print("[FPV.capture_thread] Starting video capture loop...")
    while True:
        frame_image = picam2.capture_array()
        ...
```

### Vorteile dieser Lösung

✅ **Robust:** Automatische Wiederherstellung bei Kamera-Timeout
✅ **Schnell:** Bei Erfolg nur 3s Verzögerung beim Boot
✅ **Benutzerfreundlich:** Kein manueller Restart nötig
✅ **Minimal invasiv:** Kleine Änderungen, kein großes Refactoring

### Nachteile

⚠️ **Verzögerung:** Bei 3 Retries kann Video-Start bis zu 30s dauern
⚠️ **Boot-Zeit:** 3s zusätzlicher Delay beim Service-Start

## Testing-Plan

### 1. Baseline-Test (vor Änderungen)

```bash
# 5x Reboots, jedes Mal testen:
for i in 1 2 3 4 5; do
    echo "=== Reboot Test $i/5 ==="
    sudo reboot
    # Nach Reboot:
    sleep 30
    journalctl -u gui_server.service -n 50 | grep -i camera
    # Video-Stream testen (Client verbinden)
done
```

**Erwartung:** 2-3 von 5 Tests schlagen fehl (kein Video nach erstem Start).

### 2. Mit ExecStartPre Delay (3s)

```bash
# Service-File anpassen, 5x Reboots testen
```

**Erwartung:** Erfolgsrate steigt auf 4-5 von 5.

### 3. Mit Retry-Logik

```bash
# FPV.py anpassen, 5x Reboots testen
```

**Erwartung:** 5 von 5 erfolgreich (dank Retry).

### 4. Mit beiden Änderungen

```bash
# Beide Änderungen aktiv, 10x Reboots testen
```

**Erwartung:** 10 von 10 erfolgreich, schnellster Video-Start.

## Verwandte Dokumentation

- **FT48:** Fix VIDEO_READY Race Condition (Client-Server Timing)
- **FT42:** Power Management Servo and Camera Standby (FPV.pause_stream)
- **FT12:** FPV Thread Fix (Thread läuft kontinuierlich, nicht per Client)
- **Debian Trixie Bug:** picam2.start() kann nach Reboot hängen

## Status

⏳ **In Analyse** (2026-02-16)

**Nächste Schritte:**
1. Diagnose-Logs vom Raspberry Pi sammeln
2. Timing-Messungen durchführen
3. Empfohlene Lösung (Option 3 + 2) implementieren
4. Testing durchführen (5-10 Reboots)
5. Dokumentation aktualisieren mit Ergebnissen

---

**Branch:** master (noch keine Änderungen committed)
**Author:** GitHub Copilot CLI
