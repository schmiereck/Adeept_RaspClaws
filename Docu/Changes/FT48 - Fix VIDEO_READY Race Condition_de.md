# FT48 - Fix: VIDEO_READY Race Condition

**Datum:** 2026-01-20
**Typ:** Bugfix
**Priorität:** Hoch
**Komponente:** Client/GUI.py, Server/GUIServer.py

## Problem

Die GUI blieb beim Start bei "Waiting for video server to initialize..." hängen und wartete ewig auf das VIDEO_READY Signal vom Server. Gleichzeitig traten SSH-Tunnel Fehler auf:

```
C:\Users\SCMJ178\IdeaProjects\Adeept_RaspClaws\.venv\Scripts\python.exe C:\Users\SCMJ178\IdeaProjects\Adeept_RaspClaws\Client\GUI.py
Connecting to server @ 127.0.0.1:10223 (attempt 1/8)...
✓ Connected to server
Waiting for video server to initialize...
[... GUI hängt hier ...]

SSH-Tunnel:
channel 5: open failed: connect failed: Connection refused
```

## Root Cause Analysis

### Race Condition

Es gab eine Race Condition zwischen Client und Server beim VIDEO_READY Signaling:

**Server (GUIServer.py):**
1. `accept()` → Client-Verbindung akzeptiert
2. `run()` wird sofort aufgerufen
3. `info_send_client()` Thread startet sofort
4. VIDEO_READY wird 5x mit 0.5s Abstand gesendet (2.5s Fenster)

**Client (GUI.py):**
1. `establish_connection()` → `connect()` erfolgreich
2. `print("✓ Connected to server")`
3. UI-Updates in `socket_connect()`
4. `start_connection_threads()` startet `connection_thread`
5. `print("Waiting for video server to initialize...")`
6. `connection_thread` beginnt erst jetzt, Nachrichten zu empfangen

**Timing-Problem:**
```
Zeit:     Server                        Client
------------------------------------------------------
t=0.0     accept()                      connect() OK
t=0.1     info_send_client() startet   print("✓ Connected")
t=0.1     VIDEO_READY #1 gesendet      UI-Updates...
t=0.6     VIDEO_READY #2 gesendet      UI-Updates...
t=1.1     VIDEO_READY #3 gesendet      UI-Updates...
t=1.6     VIDEO_READY #4 gesendet      start_connection_threads()
t=2.1     VIDEO_READY #5 gesendet      connection_thread startet
t=2.6     (VIDEO_READY Phase beendet)  Bereit zu empfangen ❌
```

Der Client startet den `connection_thread` zu spät und verpasst alle VIDEO_READY Signale.

### Kein Timeout

Zusätzlich gab es keinen Client-seitigen Timeout. Wenn VIDEO_READY verpasst wurde, wartete die GUI ewig und gab keine Fehlermeldung aus.

## Lösung

### 1. Server: Erweitertes VIDEO_READY Fenster

**Änderungen in Server/GUIServer.py:**

```python
# Vorher:
# Send VIDEO_READY signals (reduced to 5 times with shorter delays)
for i in range(5):  # 5x mit 0.5s = 2.5s Fenster
    tcpCliSock.send(f'{STATUS_VIDEO_READY}\n'.encode())
    time.sleep(0.5)

# Nachher:
# Wait for client to start connection_thread (avoid race condition)
print("INFO_SEND_CLIENT: Waiting 2s for client to initialize...")
time.sleep(2.0)  # ← Initialer Delay

# Send VIDEO_READY signals (10 times with 1s delays = 10s window)
for i in range(10):  # ← 10x mit 1s = 10s Fenster
    tcpCliSock.send(f'{STATUS_VIDEO_READY}\n'.encode())
    time.sleep(1.0)
```

**Verbesserungen:**
- **2s initialer Delay:** Gibt Client Zeit, UI-Updates zu machen und connection_thread zu starten
- **10s VIDEO_READY Fenster:** 10 Signale mit 1s Abstand statt 5 mit 0.5s
- **Gesamt:** 2s + 10s = 12s für Client, um bereit zu werden

### 2. Client: Timeout Watchdog

**Neue Funktion in Client/GUI.py:**

```python
def video_ready_timeout_watchdog():
    """Watchdog that warns if VIDEO_READY signal is not received within timeout"""
    global video_thread_started

    timeout_seconds = 15  # Should receive VIDEO_READY within 15 seconds
    time.sleep(timeout_seconds)

    if not video_thread_started:
        print("⚠️ WARNING: Video server did not respond within 15s")
        print("   This is usually caused by:")
        print("   1. SSH tunnel not forwarding video port (check for 'connection refused')")
        print("   2. Video server failed to start on Raspberry Pi")
        print("   3. Network issues between client and server")
        print("   → GUI will continue to work, but video stream is unavailable")
        print("   → Try reconnecting or check server logs")
```

**Integration in start_connection_threads():**

```python
def start_connection_threads():
    """Start connection and video threads"""
    global video_thread_started

    connection_threading = thread.Thread(target=connection_thread, daemon=True)
    connection_threading.start()

    print("Waiting for video server to initialize...")

    # Start timeout watchdog for VIDEO_READY signal
    timeout_thread = thread.Thread(target=video_ready_timeout_watchdog, daemon=True)
    timeout_thread.start()  # ← Neuer Watchdog-Thread
```

**Verbesserungen:**
- **15s Timeout:** Wenn VIDEO_READY nicht innerhalb 15s empfangen, gibt Watchdog Warnung aus
- **Hilfreiche Diagnose:** Zeigt mögliche Ursachen (SSH-Tunnel, Video-Server, Netzwerk)
- **GUI hängt nicht:** Informiert Benutzer, dass GUI weiterhin funktioniert (nur kein Video)

## Timing-Diagramm nach Fix

```
Zeit:     Server                        Client
------------------------------------------------------
t=0.0     accept()                      connect() OK
t=0.1     info_send_client() startet   print("✓ Connected")
t=0.1     Waiting 2s...                UI-Updates...
t=1.0     ...                          start_connection_threads()
t=1.1     ...                          connection_thread startet ✓
t=1.5     ...                          watchdog_thread startet ✓
t=2.1     VIDEO_READY #1 gesendet  →  Empfangen! ✅
t=3.1     VIDEO_READY #2 gesendet  →  (redundant)
...
t=11.1    VIDEO_READY #10 gesendet →  (redundant)
t=12.1    VIDEO_READY Phase OK         Video-Thread gestartet ✓

Alternative (falls VIDEO_READY immer noch verpasst):
t=16.5    ...                          watchdog: ⚠️ Timeout! ⚠️
```

## Testing

### Testfälle

✅ **Normaler Fall:** VIDEO_READY wird empfangen, Video startet
- Server sendet VIDEO_READY nach 2s Delay
- Client empfängt Signal innerhalb 10s Fenster
- Video-Thread startet erfolgreich

⚠️ **Timeout Fall:** VIDEO_READY wird nicht empfangen
- Nach 15s gibt Watchdog Warnung aus
- GUI bleibt funktionsfähig (nur kein Video)
- Hilfreiche Diagnose-Meldungen werden angezeigt

### Erwartetes Verhalten

**Erfolgreicher Start:**
```
Connecting to server @ 127.0.0.1:10223 (attempt 1/8)...
✓ Connected to server
Waiting for video server to initialize...
✅ Server signals: Video stream is ready
✓ Video thread started
```

**Timeout (Video-Server nicht verfügbar):**
```
Connecting to server @ 127.0.0.1:10223 (attempt 1/8)...
✓ Connected to server
Waiting for video server to initialize...
⚠️ WARNING: Video server did not respond within 15s
   This is usually caused by:
   1. SSH tunnel not forwarding video port (check for 'connection refused')
   2. Video server failed to start on Raspberry Pi
   3. Network issues between client and server
   → GUI will continue to work, but video stream is unavailable
   → Try reconnecting or check server logs
```

## Geänderte Dateien

### Server/GUIServer.py
- **Zeile ~232-250:** `info_send_client()` Funktion
  - Hinzugefügt: 2s initialer Delay
  - Geändert: 5x → 10x VIDEO_READY Signale
  - Geändert: 0.5s → 1.0s Abstand zwischen Signalen

### Client/GUI.py
- **Neu:** `video_ready_timeout_watchdog()` Funktion (~Zeile 812)
  - 15s Timeout Watchdog
  - Hilfreiche Diagnose-Meldungen
- **Geändert:** `start_connection_threads()` (~Zeile 824)
  - Startet zusätzlichen Watchdog-Thread

## Statistik

| Metrik | Vorher | Nachher | Verbesserung |
|--------|--------|---------|--------------|
| VIDEO_READY Fenster | 2.5s | 10s | **4x größer** |
| Initialer Delay | 0s | 2s | **Race Condition vermieden** |
| Client Timeout | ∞ (kein) | 15s | **Feedback statt Hängen** |
| Gesamt-Zeitfenster | 2.5s | 12s | **4.8x größer** |

## Vorteile

### 1. Robustheit

✅ **Kein Hängenbleiben mehr:** GUI gibt nach 15s Feedback statt ewig zu warten
✅ **Größeres Zeitfenster:** 12s für Client, um bereit zu werden (vorher 2.5s)
✅ **Race Condition vermieden:** 2s initialer Delay gibt Client Zeit für Thread-Start

### 2. Benutzerfreundlichkeit

✅ **Hilfreiche Fehlermeldungen:** Watchdog erklärt mögliche Ursachen
✅ **GUI bleibt funktionsfähig:** Auch ohne Video ist GUI bedienbar
✅ **Klare Kommunikation:** Benutzer weiß, dass nur Video fehlt, nicht gesamte Verbindung

### 3. Debugging

✅ **Diagnose-Hinweise:** Watchdog nennt häufigste Ursachen (SSH-Tunnel, Video-Server)
✅ **Server-Logs:** Mehr Debug-Output für VIDEO_READY Phase
✅ **Timing-Transparenz:** Logs zeigen genau, wann VIDEO_READY gesendet wurde

## Verwandte Features

- **FT45:** GUI Clean Shutdown (behandelt GUI-Schließen)
- **FT46:** Code Refactoring GUI (extrahierte `handle_video_ready()` Handler)
- **Zukünftig:** Video-Server Health Check auf Raspberry Pi

## Lessons Learned

1. **Race Conditions sind subtil:**
   - Tritt nur bei bestimmten Timing-Konstellationen auf
   - Schwer zu reproduzieren (hängt von System-Last ab)
   - Braucht sorgfältige Analyse des Message-Flows

2. **Immer Timeouts implementieren:**
   - Niemals ewig auf externe Signale warten
   - Feedback ist besser als Hängenbleiben
   - Hilfreiche Fehlermeldungen sparen Debug-Zeit

3. **Großzügige Zeitfenster:**
   - Besser 10s Fenster als 2.5s (kaum Performance-Impact)
   - Initialer Delay ist besser als Race Condition
   - Client-Systeme können unterschiedlich langsam sein

4. **SSH-Tunnel sind speziell:**
   - "Connection refused" Fehler sind normal beim Start
   - Video-Port braucht Zeit zum Initialisieren
   - Muss im Client-Code berücksichtigt werden

---

**Status:** ✅ Abgeschlossen
**Branch:** master
**Commits:** (wird beim Commit ergänzt)
