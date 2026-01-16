# Änderungen zur Behebung des Verbindungsproblems

## Problem
Der Server (`GUIServer.py`) hat versucht, eine **neue TCP-Verbindung zurück zum Client** auf Port 2256 aufzubauen, um CPU/RAM-Informationen zu senden. Diese Verbindung wurde von der Windows-Firewall blockiert.

## Lösung
Die CPU/RAM-Informationen werden jetzt über die **bestehende Verbindung** gesendet, die der Client zum Server aufgebaut hat.

---

## Geänderte Dateien

### 1. **Server/GUIServer.py**

**Funktion `info_send_client()` (Zeile ~87-100):**
- ❌ **Vorher:** Versuch, neue Verbindung zum Client auf Port 2256 aufzubauen
- ✅ **Jetzt:** Verwendet die bestehende `tcpCliSock`-Verbindung
- Die Info-Daten werden mit dem Prefix `INFO:` markiert

```python
def info_send_client():
    # Use the existing tcpCliSock connection instead of creating a new one
    while 1:
        try:
            info_data = 'INFO:' + get_cpu_tempfunc() + ' ' + get_cpu_use() + ' ' + get_ram_info() + '\n'
            tcpCliSock.send(info_data.encode())
            time.sleep(1)
        except:
            pass
```

---

### 2. **Client/GUI.py**

**A) Funktion `connection_thread()` (Zeile ~301):**
- ✅ **Neu:** Verarbeitung von `INFO:`-Nachrichten hinzugefügt
- Die CPU/RAM-Daten werden direkt im bestehenden Thread verarbeitet

```python
elif car_info.startswith('INFO:'):
    # Process CPU/RAM info from server
    try:
        info_data = car_info[5:].strip()  # Remove 'INFO:' prefix
        info_get = info_data.split()
        if len(info_get) >= 3:
            CPU_TEP, CPU_USE, RAM_USE = info_get[0], info_get[1], info_get[2]
            CPU_TEP_lab.config(text='CPU Temp: %s℃'%CPU_TEP)
            CPU_USE_lab.config(text='CPU Usage: %s'%CPU_USE)
            RAM_lab.config(text='RAM Usage: %s'%RAM_USE)
    except:
        pass
```

**B) Funktion `Info_receive()` (Zeile ~382-404):**
- ❌ **Entfernt:** Die separate Funktion wird nicht mehr benötigt

**C) Thread-Start für `Info_receive()` (Zeile ~450-453):**
- ❌ **Entfernt:** Der separate Thread wird nicht mehr gestartet

---

## Vorteile
✅ Keine zusätzliche Verbindung nötig  
✅ Funktioniert auch hinter Firewalls/NAT  
✅ Einfachere Architektur  
✅ Port 2256 muss nicht mehr geöffnet werden  

---

## Nächste Schritte
1. **Änderungen committen und pushen:**
   ```bash
   git add Server/GUIServer.py Client/GUI.py Docu/Changes/FT1 - Fix Connection Problem_de.md Docu/Changes/FT1 - Fix Connection Problem_en.md
   git commit -m "Fix: Use existing connection for CPU/RAM info instead of opening new connection"
   git push
   ```

2. **Auf dem Raspberry Pi:**
   ```bash
   git clone https://github.com/schmiereck/Adeept_RaspClaws.git
   cd Adeept_RaspClaws
   git pull
   ```

3. **Testen:**
   - Client auf Windows starten
   - Server auf Raspberry Pi starten
   - CPU/RAM-Informationen sollten jetzt angezeigt werden
