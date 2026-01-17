# Fix: Video-Stream funktioniert nicht

## Problem

Der Client verbindet sich erfolgreich mit dem Video-Stream-Server (ZMQ auf Port 5555), aber es kommen keine Video-Frames an. Der Client zeigt permanent "Timeout waiting for video frame" an.

### Symptome
```
[Footage] Connecting to video stream @ 127.0.0.1:5555
[Footage] Connected to video stream, waiting for frames...
[Footage] Starting video receive loop...
[Footage] ⚠ Timeout waiting for video frame
[Footage] ⚠ Timeout waiting for video frame
...
```

## Root Cause

In `Server/FPV.py` war die Frame-Encoding und Sende-Logik **außerhalb** der `while True:` Schleife eingerückt (Zeilen 385-395). Das bedeutete, dass diese Zeilen nie ausgeführt wurden, da sie auf der gleichen Einrückungsebene wie die Schleife selbst lagen.

### Fehlerhafte Struktur
```python
while True:
    frame_image = picam2.capture_array()
    # ... Bildverarbeitung ...
    
if FindLineMode:  # ❌ FALSCH: Außerhalb der Schleife!
    encoded, buffer = cv2.imencode('.jpg', frame_findline)
else:
    encoded, buffer = cv2.imencode('.jpg', frame_image)
footage_socket.send(jpg_as_text)
```

## Lösung

Die Frame-Encoding und Sende-Logik wurde korrekt innerhalb der `while True:` Schleife eingerückt.

### Datei: `Server/FPV.py` (Zeilen 385-395)

**Vorher:**
```python
        if (timestamp - lastMovtionCaptured).seconds >= 0.5:
            switch.switch(1,0)
            switch.switch(2,0)
            switch.switch(3,0)

if FindLineMode:  # ❌ Außerhalb der Schleife
    frame_findline = cvFindLine(frame_image)
    encoded, buffer = cv2.imencode('.jpg', frame_findline)
else:
    encoded, buffer = cv2.imencode('.jpg', frame_image)
jpg_as_text = base64.b64encode(buffer)
footage_socket.send(jpg_as_text)
time.sleep(0.033)
```

**Nachher:**
```python
        if (timestamp - lastMovtionCaptured).seconds >= 0.5:
            switch.switch(1,0)
            switch.switch(2,0)
            switch.switch(3,0)

    # Frame encoding and sending (MUST be inside the while True loop!)
    if FindLineMode:  # ✅ Innerhalb der Schleife
        frame_findline = cvFindLine(frame_image)
        encoded, buffer = cv2.imencode('.jpg', frame_findline)
    else:
        encoded, buffer = cv2.imencode('.jpg', frame_image)
    jpg_as_text = base64.b64encode(buffer)
    footage_socket.send(jpg_as_text)
    
    # Limit frame rate to reduce CPU load (~30 FPS = 33ms per frame)
    time.sleep(0.033)  # 33ms = ~30 FPS, reduces CPU load significantly
```

## Zusätzliche Änderungen

### Client/GUI.py
- `run_open()` verwendet jetzt `subprocess.Popen()` statt `subprocess.run()` für nicht-blockierenden Start des Footage-GUI Prozesses
- Ein Monitor-Thread zeigt die Ausgaben des Footage-GUI Prozesses in Echtzeit an
- Ordnungsgemäßes Cleanup des Footage-Prozesses beim Beenden

### Client/Footage-GUI.py
- Bessere Debug-Ausgaben hinzugefügt (Frame-Counter alle 30 Frames)
- Timeout-Handling mit `poll()` für bessere Fehlerdiagnose
- Fehlerbehandlung mit detaillierten Traceback-Ausgaben

## Ergebnis

Nach dieser Änderung:
- Der FPV-Thread sendet kontinuierlich Frames über ZMQ
- Der Client empfängt und zeigt die Frames an (~30 FPS)
- Die CPU-Last auf dem Raspberry Pi bleibt niedrig durch die Frame-Rate-Limitierung

## Test

1. Server auf Raspberry Pi starten: `sudo systemctl restart robot_server.service`
2. Client auf Windows starten und verbinden
3. Nach dem "VIDEO_READY" Signal sollte das Video-Fenster erscheinen und Frames anzeigen

## Verwandte Dateien
- `Server/FPV.py` - Frame-Capture und Video-Stream
- `Client/GUI.py` - Hauptanwendung mit Video-Thread-Management
- `Client/Footage-GUI.py` - Video-Anzeige-Fenster
