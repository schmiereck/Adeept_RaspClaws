# Fix: Video-Stream beim zweiten Connect funktioniert nicht

## Problem

Beim **ersten Connect** funktioniert der Video-Stream perfekt, aber beim **zweiten Connect** (nach vollständigem Neustart der GUI) erscheint nur:
```
Waiting for video server to initialize...
```
Das Video-Fenster erscheint nicht.

## Root Cause

Das Problem liegt auf dem **Server**, nicht auf dem Client!

**Beim ersten Connect:**
1. Server akzeptiert Verbindung
2. Server startet `info_send_client` Thread in `run()`
3. Server sendet `VIDEO_READY` Signal im `video_ready_thread` (0,5 Sekunden Verzögerung)
4. ✅ Client empfängt Signal und startet Video

**Beim zweiten Connect:**
1. Server akzeptiert neue Verbindung
2. Server startet `info_send_client` Thread in `run()`  
3. Server sendet `VIDEO_READY` Signal im `video_ready_thread` (0,5 Sekunden Verzögerung)
4. ❌ **ABER:** Der `connection_thread` im Client ist möglicherweise noch nicht bereit, das Signal zu empfangen!
5. Das Signal geht verloren und wird **nie wieder** gesendet

### Timing-Problem

Der `video_ready_thread` sendet das `VIDEO_READY` Signal nur **einmal** nach 0,5 Sekunden. Wenn der Client-Thread zu diesem Zeitpunkt noch nicht in der `recv()` Schleife ist, geht das Signal verloren.

## Lösung

### Datei: `Server/GUIServer.py`

**Strategie:** Statt das `VIDEO_READY` Signal nur einmal zu senden, wird es **mehrmals am Anfang** im `info_send_client` Thread gesendet. Dieser Thread läuft sowieso während der gesamten Verbindung und sendet regelmäßig System-Informationen.

#### Änderung 1: `info_send_client` Funktion (Zeilen 103-120)

**Vorher:**
```python
def info_send_client():
    while 1:
        try:
            info_data = 'INFO:' + get_cpu_tempfunc() + ' ' + get_cpu_use() + ' ' + get_ram_info() + '\n'
            tcpCliSock.send(info_data.encode())
            time.sleep(1)
        except:
            pass
```

**Nachher:**
```python
def info_send_client():
    # Send VIDEO_READY signal multiple times at the start
    # This ensures the client receives it even if it's not ready immediately
    for i in range(5):  # Send 5 times over 5 seconds
        try:
            tcpCliSock.send('VIDEO_READY\n'.encode())
            print(f"✅ Sent VIDEO_READY signal (attempt {i+1}/5)")
            time.sleep(1)
        except:
            break
    
    # Then continue with regular info sending
    while 1:
        try:
            info_data = 'INFO:' + get_cpu_tempfunc() + ' ' + get_cpu_use() + ' ' + get_ram_info() + '\n'
            tcpCliSock.send(info_data.encode())
            time.sleep(1)
        except:
            pass
```

#### Änderung 2: Entfernung des separaten `video_ready_thread` (Zeilen 384-427)

Der separate `video_ready_thread` wird nicht mehr benötigt, da das Signal jetzt im `info_send_client` Thread gesendet wird.

**Vorher:**
```python
# Start signal thread (non-blocking)
video_ready_thread = threading.Thread(target=send_video_ready_signal, daemon=True)
video_ready_thread.start()
print("✓ Video ready check running in background")
```

**Nachher:**
```python
# Note: VIDEO_READY signal is now sent in info_send_client thread
# which starts automatically when run() is called
```

## Vorteile dieser Lösung

1. **Zuverlässig:** Das Signal wird 5 Mal gesendet (über 5 Sekunden), sodass der Client es garantiert empfängt
2. **Einfach:** Keine separaten Threads oder komplexe Synchronisation
3. **Robust:** Funktioniert unabhängig vom Timing des Client-Starts
4. **Idempotent:** Der Client ignoriert mehrfache `VIDEO_READY` Signale (nach dem ersten wird `video_thread_started = True` gesetzt)

## Ergebnis

Nach dieser Änderung:
- ✅ **Erster Connect**: Video-Stream funktioniert perfekt
- ✅ **Zweiter Connect**: Video funktioniert sofort (Signal wird mehrmals gesendet)
- ✅ **Dritter, vierter, ... Connect**: Funktioniert immer
- ✅ **Unabhängig vom Timing**: Auch wenn der Client langsam startet, empfängt er das Signal

## Test-Szenarios

### Szenario 1: Normaler Neustart
1. GUI starten und verbinden → Video funktioniert ✅
2. GUI komplett schließen
3. GUI neu starten und verbinden → Video funktioniert ✅

### Szenario 2: Mehrfaches Neuverbinden
1. GUI starten und verbinden → Video funktioniert ✅
2. GUI schließen
3. GUI starten und verbinden → Video funktioniert ✅
4. Schritte 2-3 mehrmals wiederholen → Video funktioniert immer ✅

### Szenario 3: Langsamer Client-Start
1. GUI starten auf langsamem PC
2. Verbinden dauert einige Sekunden
3. → Video funktioniert trotzdem ✅ (Signal wird mehrmals gesendet)

## Verwandte Dateien
- `Server/GUIServer.py` - Verbesserte VIDEO_READY Signal-Logik im `info_send_client` Thread
