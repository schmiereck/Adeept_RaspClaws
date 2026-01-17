# Fix: Zweiter Connect funktioniert nicht - Server akzeptiert keine neuen Verbindungen

## Problem

Beim **ersten Connect** funktioniert alles perfekt (Video-Stream funktioniert). Aber beim **zweiten Connect** (nach GUI-Neustart):
- GUI zeigt: "Connected" und "Waiting for video server to initialize..."
- Server-Logs zeigen: **Keine** neue Verbindung
- Kein Video-Fenster erscheint
- GUI hängt und wartet ewig

## Root Cause

Es gab **zwei kritische Bugs** im Server-Code:

### Bug #1: `continue` statt `break` bei leerem Socket (HAUPTPROBLEM!)

**Datei:** `Server/GUIServer.py`, Zeile 168-169 (Funktion `run()`)

**Fehlerhafter Code:**
```python
data = str(tcpCliSock.recv(BUFSIZ).decode())
if not data:
    continue  # ❌ FALSCH!
```

**Problem:**
Wenn ein Client disconnected, gibt `recv()` einen **leeren String** zurück. Der Code macht dann `continue`, was bedeutet, dass die `while True:` Schleife **weiter läuft** und ewig auf Daten vom toten Socket wartet!

**Folge:**
- Erster Connect: Funktioniert, dann GUI schließen
- `recv()` gibt leeren String zurück
- Code macht `continue` → **Schleife läuft weiter**
- `run()` kehrt **NIE** zurück
- Zweiter Connect: Server kann **KEINE** neue Verbindung akzeptieren, weil er noch in der alten `run()` Schleife hängt!

**Lösung:**
```python
data = str(tcpCliSock.recv(BUFSIZ).decode())
if not data:
    # Empty string means socket was closed by client
    print("Client disconnected (empty recv)")
    sys.stdout.flush()
    break  # ✅ Exit loop to wait for new connection
```

Mit `break` verlässt die Funktion `run()` die Schleife, kehrt zurück zur Hauptschleife, schließt die Sockets und akzeptiert eine neue Verbindung.

### Bug #2: `VIDEO_READY` Signal ging beim zweiten Connect verloren

**Datei:** `Server/GUIServer.py`, Funktion `info_send_client()`

**Probleme in der ursprünglichen Version:**
1. Signal wurde nur **5 Mal** gesendet (zu wenig bei langsamen Clients)
2. Bei einem Fehler wurde sofort `break` aufgerufen → keine weiteren Versuche
3. Keine Debug-Ausgaben → unmöglich zu debuggen
4. `print()` war gepuffert → Logs erschienen nicht sofort

**Lösung:**
```python
def info_send_client():
    print("INFO_SEND_CLIENT: Starting to send VIDEO_READY signals...")
    sys.stdout.flush()  # Force output immediately
    
    success_count = 0
    for i in range(10):  # Try 10 times (increased from 5)
        try:
            tcpCliSock.send('VIDEO_READY\n'.encode())
            success_count += 1
            print(f"✅ Sent VIDEO_READY signal (attempt {i+1}/10, success #{success_count})")
            sys.stdout.flush()
            time.sleep(1)
        except Exception as e:
            print(f"⚠ Failed to send VIDEO_READY (attempt {i+1}/10): {e}")
            sys.stdout.flush()
            # Don't break - continue trying!  ← WICHTIG!
            time.sleep(1)
    
    print(f"INFO_SEND_CLIENT: Finished VIDEO_READY phase ({success_count}/10 successful)")
    sys.stdout.flush()
    
    # Then continue with regular info sending
    while 1:
        try:
            info_data = 'INFO:' + get_cpu_tempfunc() + ' ' + get_cpu_use() + ' ' + get_ram_info() + '\n'
            tcpCliSock.send(info_data.encode())
            time.sleep(1)
        except Exception as e:
            print(f"⚠ Failed to send INFO: {e}")
            sys.stdout.flush()
            break  # Exit thread on persistent error
```

**Verbesserungen:**
- 10 Versuche statt 5
- Bei Fehler wird **nicht** abgebrochen, sondern weiter versucht
- Ausführliche Debug-Ausgaben mit `sys.stdout.flush()`
- Zählt erfolgreiche Sends

## Änderungen im Detail

### Datei: `Server/GUIServer.py`

**Zeile 11:** Import von `sys` hinzugefügt
```python
import sys
```

**Zeile 106-137:** Verbesserung der `info_send_client()` Funktion
- Mehr Versuche (10 statt 5)
- Bessere Fehlerbehandlung
- Debug-Ausgaben mit `sys.stdout.flush()`

**Zeile 168-172:** Kritischer Bugfix in `run()` Funktion
```python
if not data:
    # Empty string means socket was closed by client
    print("Client disconnected (empty recv)")
    sys.stdout.flush()
    break  # ← GEÄNDERT von 'continue' zu 'break'
```

## Wie die Fehler gefunden wurden

### Phase 1: Analyse der Logs
Server-Logs zeigten beim zweiten Connect:
- **Keine** "waiting for connection..." Meldung
- **Keine** "...connected from..." Meldung
- **Keine** VIDEO_READY Signale

→ **Schlussfolgerung:** Server akzeptiert keine neue Verbindung!

### Phase 2: Code-Analyse
Die Haupt-Schleife im `__main__` Block:
```python
while 1:
    # Accept connection
    tcpCliSock, addr = tcpSerSock.accept()
    print('...connected from :', addr)
    
    # Run command processing
    run()  # ← Hängt hier beim zweiten Connect!
    
    # Cleanup and loop back
    print("Client disconnected. Cleaning up...")
```

Die `run()` Funktion sollte zurückkehren, wenn der Client disconnected. **ABER:** Die Logs zeigten nie "Client disconnected. Cleaning up..."

→ **Schlussfolgerung:** `run()` kehrt nicht zurück!

### Phase 3: `run()` Analyse
```python
def run():
    while True:
        data = str(tcpCliSock.recv(BUFSIZ).decode())
        if not data:
            continue  # ← HIER IST DER BUG!
```

Wenn `recv()` leeren String zurückgibt (Socket geschlossen), macht `continue` → Schleife läuft ewig weiter!

## Server-Architektur (zur Klarstellung)

```
Server Start
    │
    ├─ Start FPV Thread (läuft kontinuierlich, sendet Video-Frames)
    │
    └─ Haupt-Schleife (while 1):
        │
        ├─ Akzeptiere neue Verbindung: tcpSerSock.accept()
        │
        ├─ Rufe run() auf
        │   │
        │   ├─ Starte info_send_client Thread
        │   │   └─ Sende 10x VIDEO_READY
        │   │   └─ Sende INFO-Nachrichten
        │   │
        │   └─ Command Loop (while True):
        │       └─ Empfange und verarbeite Befehle
        │       └─ Bei Disconnect: break → zurück zur Haupt-Schleife
        │
        ├─ Cleanup (Sockets schließen)
        │
        └─ Zurück zu Schritt 1 (neue Verbindung akzeptieren)
```

**Mit dem Bug:** `run()` kehrt nie zurück → keine neue Verbindung möglich
**Mit dem Fix:** `run()` kehrt zurück → neue Verbindung kann akzeptiert werden ✅

## Ergebnis

Nach diesem Fix:
- ✅ **Erster Connect**: Video funktioniert
- ✅ **Zweiter Connect**: Video funktioniert
- ✅ **Dritter, vierter, ... Connect**: Funktioniert immer
- ✅ Server akzeptiert beliebig viele aufeinanderfolgende Verbindungen

## Test-Szenarien

### Szenario 1: Mehrfaches GUI-Restart
1. GUI starten und verbinden → Video funktioniert ✅
2. GUI schließen
3. Server-Logs zeigen: "Client disconnected (empty recv)"
4. Server-Logs zeigen: "Client disconnected. Cleaning up..."
5. Server-Logs zeigen: "Ready for new connection..."
6. GUI neu starten und verbinden → Video funktioniert ✅
7. Schritte 2-6 beliebig oft wiederholen → Funktioniert immer ✅

### Szenario 2: Server-Neustart
1. Server neu starten
2. GUI verbinden → Video funktioniert ✅
3. GUI schließen und neu starten → Video funktioniert ✅

### Szenario 3: Netzwerk-Unterbrechung
1. GUI verbunden, Video läuft
2. Netzwerk-Unterbrechung (z.B. WLAN kurz weg)
3. `run()` wirft Exception → `break` → Cleanup
4. Netzwerk wieder da
5. GUI neu verbinden → Video funktioniert ✅

## Verwandte Dateien
- `Server/GUIServer.py` - Haupt-Server mit kritischem Bugfix
- `Client/GUI.py` - Client-Anwendung (keine Änderungen nötig)
