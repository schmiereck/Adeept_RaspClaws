# FT45 - GUI Clean Shutdown

**Datum:** 2026-01-20  
**Typ:** Bugfix  
**Komponente:** Client/GUI.py

## Problem

Beim Schließen des GUI-Fensters wurde die Anwendung nicht ordnungsgemäß beendet:
- Die TCP-Socket-Verbindung blieb offen
- Der Footage-GUI Prozess lief weiter
- Der ZMQ Footage-Socket wurde nicht geschlossen
- **Die Anwendung lief im Hintergrund weiter, obwohl das Fenster geschlossen wurde**
- Bei nachfolgenden Befehlen entstanden Fehlermeldungen wie:
  ```
  Exception in Tkinter callback
  Traceback (most recent call last):
    File "...\tkinter\__init__.py", line 1705, in __call__
      return self.func(*args)
    File "...\GUI.py", line 175, in call_forward
      send_movement_command('forward', 'c_f_stu')
    File "...\GUI.py", line 167, in send_movement_command
      send_command(command)
    File "...\GUI.py", line 149, in send_command
      tcpClicSock.send(command.encode())
  ```

## Lösung

### 1. on_closing() Funktion hinzugefügt
Eine neue Funktion wurde implementiert, die beim Schließen des Fensters aufgerufen wird und alle Ressourcen ordnungsgemäß freigibt:

```python
def on_closing():
    """Clean shutdown when window is closed"""
    global tcpClicSock, footage_socket, footage_process, shutdown_requested
    
    shutdown_requested = True  # Signal that shutdown was requested
    print("Shutting down GUI...")
    
    # TCP Socket schließen
    if tcpClicSock and tcpClicSock != '':
        tcpClicSock.close()
    
    # Footage Socket schließen
    if footage_socket is not None:
        footage_socket.close()
    
    # Footage-GUI Prozess beenden
    if footage_process is not None and footage_process.poll() is None:
        footage_process.terminate()
        footage_process.wait(timeout=2)
    
    # OpenCV Fenster schließen
    cv2.destroyAllWindows()
    
    # GUI Fenster schließen
    root.destroy()
    
    # Anwendung vollständig beenden
    import sys
    sys.exit(0)
```

### 2. shutdown_requested Flag hinzugefügt
Ein globales Flag wurde hinzugefügt, um zu signalisieren, dass die Anwendung beendet werden soll:

```python
shutdown_requested = False  # Flag to signal application shutdown
```

### 3. While-Schleife angepasst
Die while-Schleife in der `loop()` Funktion wurde erweitert, um nach `root.mainloop()` zu überprüfen, ob ein Shutdown angefordert wurde:

```python
if shutdown_requested:
    print("Exiting application loop...")
    break
```

### 2. Window Close Protocol registriert
Die on_closing() Funktion wird beim Schließen des Fensters automatisch aufgerufen:

```python
root.protocol("WM_DELETE_WINDOW", on_closing)
```

### 3. Fehlerbehandlung in send_command() verbessert
Die send_command() Funktion prüft nun, ob der Socket noch gültig ist, bevor versucht wird zu senden:

```python
def send_command(command):
    """Send a command to the server"""
    try:
        if tcpClicSock:
            tcpClicSock.send(command.encode())
    except Exception as e:
        print(f"Error sending command '{command}': {e}")
```

### 4. connection_thread() mit Fehlerbehandlung
Der Connection-Thread beendet sich nun ordnungsgemäß, wenn der Socket geschlossen wird:

```python
def connection_thread():
    try:
        while 1:
            car_info = (tcpClicSock.recv(BUFSIZ)).decode()
            # ... Verarbeitung ...
    except Exception as e:
        print(f"Connection thread stopped: {e}")
```

### 5. Konsistente Verwendung von send_command()
Die Funktionen `call_servo_standby()` und `call_camera_pause()` verwenden nun die zentrale send_command() Funktion statt direkter Socket-Aufrufe.

## Änderungen

**Client/GUI.py:**
- `shutdown_requested` Flag hinzugefügt (Zeile ~68)
- `on_closing()` Funktion hinzugefügt mit `sys.exit(0)` (Zeile ~763-812)
- `root.protocol("WM_DELETE_WINDOW", on_closing)` registriert (Zeile ~814)
- `shutdown_requested` zur globalen Variablenliste der `loop()` Funktion hinzugefügt (Zeile ~748)
- While-Schleife erweitert mit Shutdown-Check und `break` (Zeile ~986-989)
- `send_command()` mit Fehlerbehandlung erweitert (Zeile ~149-155)
- `connection_thread()` mit try-except umschlossen (Zeile ~507-642)
- Einrückung im VIDEO_READY Block korrigiert (Zeile ~515-522)
- `call_servo_standby()` verwendet jetzt `send_command()` (Zeile ~327-340)
- `call_camera_pause()` verwendet jetzt `send_command()` (Zeile ~343-356)

## Testergebnisse

### Vor der Änderung:
- ❌ Fenster schließen führt zu Tkinter-Fehlermeldungen
- ❌ Footage-GUI Prozess läuft nach Beenden weiter
- ❌ Socket-Verbindungen bleiben offen

### Nach der Änderung:
- ✅ Fenster schließt sauber ohne Fehlermeldungen
- ✅ Alle Prozesse werden ordnungsgemäß beendet
- ✅ Alle Socket-Verbindungen werden geschlossen
- ✅ OpenCV-Fenster werden geschlossen

## Auswirkungen

- **Benutzerfreundlichkeit:** ✅ Verbessert - keine Fehlermeldungen mehr beim Schließen
- **Stabilität:** ✅ Verbessert - sauberes Shutdown-Verhalten
- **Performance:** ✅ Verbessert - keine Zombie-Prozesse mehr
- **Rückwärtskompatibilität:** ✅ Vollständig gegeben

## Offene Punkte

Keine.

## Notizen

Die Lösung folgt dem Best-Practice-Muster für Tkinter-Anwendungen, bei dem das WM_DELETE_WINDOW Protokoll verwendet wird, um eine saubere Cleanup-Routine beim Schließen des Fensters auszuführen.
