# FT50 - Installation auf Raspberry Pi

**Quick Installation Guide**

## Schritt 1: Code aktualisieren

```bash
ssh pi@192.168.2.126
cd /home/pi/Adeept_RaspClaws
git pull
```

## Schritt 2: Service-File aktualisieren

```bash
# Backup des alten Service-Files erstellen
sudo cp /etc/systemd/system/gui_server.service /etc/systemd/system/gui_server.service.backup

# Neues Service-File installieren
sudo cp systemd/gui_server.service /etc/systemd/system/

# systemd neu laden
sudo systemctl daemon-reload
```

## Schritt 3: Service neu starten

```bash
# Service neu starten (FPV.py Änderungen werden geladen)
sudo systemctl restart gui_server.service

# Logs beobachten
sudo journalctl -u gui_server.service -f
```

**Erwartete Ausgabe:**
```
Starting ROS2 Adeept RaspClaws GUI Server...
[3 Sekunden Pause - ExecStartPre]
[GUIServer] Starting...
[FPV] Initializing camera with retry logic...
[FPV.capture_thread] Camera init attempt 1/3
[FPV.capture_thread]   Creating Picamera2 object...
[FPV.capture_thread]   Configuring camera...
[FPV.capture_thread]   Starting camera with 5-second timeout...
[FPV.capture_thread] ✓ Camera started successfully
[FPV.capture_thread] Starting video capture loop...
```

## Schritt 4: Testing

### Test 1: Client verbinden

```bash
# Auf Windows:
# 1. Client GUI starten
# 2. Verbinden zu 192.168.2.126
# 3. Video-Stream sollte funktionieren
```

### Test 2: Service Status prüfen

```bash
sudo systemctl status gui_server.service
```

Sollte zeigen:
```
● gui_server.service - ROS2 Adeept RaspClaws GUI Server
   Active: active (running) since ...
   Main PID: ...
```

### Test 3: Reboot-Test

```bash
# Pi neu starten
sudo reboot

# Nach Reboot (~30s warten)
# Logs prüfen:
sudo journalctl -u gui_server.service -n 100 | grep -E "Camera|attempt|✓|❌"

# Client verbinden und Video testen
```

**Erwartetes Ergebnis:** Video startet automatisch ohne manuellen Restart

## Rollback bei Problemen

```bash
# Altes Service-File wiederherstellen
sudo cp /etc/systemd/system/gui_server.service.backup /etc/systemd/system/gui_server.service
sudo systemctl daemon-reload
sudo systemctl restart gui_server.service

# Code zurücksetzen
cd /home/pi/Adeept_RaspClaws
git checkout HEAD~1 Server/FPV.py
sudo systemctl restart gui_server.service
```

## Nützliche Befehle

```bash
# Service-Logs (letzte 50 Zeilen)
sudo journalctl -u gui_server.service -n 50 --no-pager

# Service-Logs live folgen
sudo journalctl -u gui_server.service -f

# Service Status
sudo systemctl status gui_server.service

# Service neu starten
sudo systemctl restart gui_server.service

# Service stoppen
sudo systemctl stop gui_server.service

# Service starten
sudo systemctl start gui_server.service

# Kamera-Initialisierung testen (manuell)
python3 -c "from picamera2 import Picamera2; p = Picamera2(); p.start(); print('OK')"
```

## Was wurde geändert?

### 1. systemd/gui_server.service
- **Neu:** `ExecStartPre=/bin/sleep 3` - wartet 3 Sekunden vor dem Start
- **Zweck:** Gibt Kamera-Hardware Zeit zum Initialisieren nach Boot

### 2. Server/FPV.py
- **Neu:** Retry-Logik mit 3 Versuchen (je 10s Delay)
- **Zweck:** Automatische Wiederherstellung bei Kamera-Timeout
- **Vorteil:** Kein manueller Service-Restart mehr nötig

## Erfolg verifizieren

Nach Installation sollte:
1. ✅ Service startet mit 3s Delay (in Logs sichtbar)
2. ✅ Kamera initialisiert beim ersten Versuch (meist)
3. ✅ Bei Timeout: Automatischer Retry nach 10s
4. ✅ Video-Stream startet automatisch
5. ✅ Nach Reboot: Video funktioniert ohne manuellen Restart

---

**Support:** Siehe `Docu/Changes/FT50 - Fix Camera Start on Reboot_de.md` für Details
