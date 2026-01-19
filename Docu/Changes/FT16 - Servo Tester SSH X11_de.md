# ServoTester über SSH ohne Monitor - Anleitung

## Übersicht

Sie können den ServoTester über **SSH mit X11-Forwarding** verwenden, ohne einen Monitor direkt am Raspberry Pi anzuschließen. Die GUI wird dann auf Ihrem Windows-PC angezeigt, während die Anwendung auf dem Pi läuft.

---

## Voraussetzungen

### Auf Windows

Sie benötigen einen **X-Server** (X11-Server für Windows):

**Empfohlene Option: VcXsrv** (kostenlos, Open Source)
- Download: https://sourceforge.net/projects/vcxsrv/
- Alternative: Xming (https://sourceforge.net/projects/xming/)
- Alternative: MobaXterm (enthält integrierten X-Server)

### Auf dem Raspberry Pi

SSH muss X11-Forwarding erlauben (normalerweise standardmäßig aktiviert).

---

## Schritt-für-Schritt Anleitung

### 1. X-Server auf Windows installieren

#### VcXsrv Installation:

1. **Download VcXsrv**:
   ```
   https://sourceforge.net/projects/vcxsrv/files/latest/download
   ```

2. **Installieren**:
   - Doppelklick auf `vcxsrv-installer.exe`
   - Standardeinstellungen akzeptieren
   - Installation durchführen

3. **Start über XLaunch**:
   - Start → XLaunch (im Startmenü)
   
4. **Konfiguration**:
   
   **Seite 1 - Display Settings**:
   ```
   ○ Multiple windows  ← Wählen Sie diese Option
   ○ One large window
   ○ One window without titlebar
   ○ Fullscreen
   
   Display number: 0  ← Standard belassen
   ```
   → **Next**
   
   **Seite 2 - Client Startup**:
   ```
   ○ Start no client  ← Wählen Sie diese Option
   ○ Start a program
   ○ Open session via XDMCP
   ```
   → **Next**
   
   **Seite 3 - Extra Settings**:
   ```
   ☑ Clipboard          ← Aktivieren (für Copy/Paste)
   ☑ Primary Selection  ← Aktivieren
   ☐ Native opengl
   ☑ Disable access control  ← WICHTIG: Aktivieren!
   ```
   → **Next**
   
   **Seite 4 - Finish**:
   ```
   Optional: "Save configuration" anklicken
   → Speichern als "servo_tester.xlaunch" auf Desktop
   ```
   → **Finish**

5. **VcXsrv läuft jetzt**:
   - Icon im System Tray (Taskleiste)
   - Läuft im Hintergrund
   - Wartet auf X11-Verbindungen

---

### 2. SSH-Verbindung mit X11-Forwarding

#### PowerShell-Befehl:

```powershell
ssh -X pi@192.168.2.126
```

**Oder detaillierter** (bei Problemen):
```powershell
ssh -Y -C pi@192.168.2.126
```

**Parameter-Erklärung**:
- `-X`: X11-Forwarding aktivieren (sicher)
- `-Y`: Trusted X11-Forwarding (weniger restriktiv, falls `-X` nicht funktioniert)
- `-C`: Kompression aktivieren (schnellere Übertragung)

#### Windows CMD Alternative:

Falls Sie keine PowerShell verwenden:
```cmd
ssh -X pi@192.168.2.126
```

#### Mit PuTTY:

Falls Sie PuTTY verwenden:

1. **PuTTY öffnen**
2. **Connection → SSH → X11**:
   - ☑ **Enable X11 forwarding** aktivieren
   - X display location: `localhost:0.0`
3. **Session**:
   - Host Name: `pi@192.168.2.126`
   - Port: `22`
   - Connection type: SSH
4. **Open** klicken

---

### 3. X11-Verbindung testen

Nach dem Einloggen auf dem Pi, testen Sie die X11-Verbindung:

```bash
# DISPLAY Variable prüfen
echo $DISPLAY
```

**Erwartete Ausgabe**:
```
localhost:10.0
```
Oder ähnlich (z.B. `localhost:11.0`). Hauptsache, es wird etwas angezeigt!

**Einfacher X11-Test**:
```bash
xclock
```

→ Es sollte ein Uhr-Fenster auf Ihrem Windows-PC erscheinen!

**Falls xclock nicht installiert ist**:
```bash
sudo apt-get update
sudo apt-get install x11-apps
xclock
```

---

### 4. ServoTester starten

Wenn der X11-Test funktioniert hat:

```bash
cd /home/pi/adeept_raspclaws/Server
python3 ServoTester.py
```

**Was passiert**:
1. Terminal zeigt: "Starting GUI..."
2. Nach 1-2 Sekunden erscheint die GUI auf Ihrem Windows-PC
3. Sie können die Slider bedienen
4. PWM-Befehle werden auf dem Pi ausgeführt
5. Terminal zeigt Debugging-Ausgaben

---

## Troubleshooting

### Problem 1: "cannot open display"

**Fehlermeldung**:
```
_tkinter.TclError: couldn't connect to display "localhost:10.0"
```

**Lösungen**:

#### A) X-Server nicht gestartet
```powershell
# Prüfen: Läuft VcXsrv?
# → Icon im System Tray suchen
# Falls nicht: XLaunch starten
```

#### B) DISPLAY Variable nicht gesetzt
```bash
# Auf dem Pi:
echo $DISPLAY
# Falls leer:
export DISPLAY=localhost:10.0
python3 ServoTester.py
```

#### C) X11-Forwarding nicht aktiviert
```bash
# Neue SSH-Verbindung mit explizitem X11:
exit
ssh -Y pi@192.168.2.126
```

---

### Problem 2: "X11 forwarding request failed"

**Ursache**: SSH-Server auf Pi erlaubt kein X11-Forwarding

**Lösung**:

```bash
# SSH-Konfiguration auf dem Pi prüfen
sudo nano /etc/ssh/sshd_config

# Diese Zeile finden und sicherstellen, dass sie so aussieht:
X11Forwarding yes
X11DisplayOffset 10
X11UseLocalhost yes

# Änderungen speichern (Ctrl+X, Y, Enter)

# SSH-Service neu starten
sudo systemctl restart ssh

# Neu einloggen
exit
ssh -X pi@192.168.2.126
```

---

### Problem 3: GUI ist sehr langsam

**Ursache**: X11-Forwarding über Netzwerk kann langsam sein

**Lösungen**:

#### A) Kompression aktivieren
```powershell
ssh -X -C pi@192.168.2.126
```

#### B) Trusted X11 verwenden
```powershell
ssh -Y pi@192.168.2.126
```

#### C) Bessere Netzwerkverbindung
- Ethernet statt WLAN nutzen
- Näher am Router/Access Point
- QoS im Router aktivieren

---

### Problem 4: Windows Firewall blockiert

**Fehlermeldung**: Keine Verbindung möglich

**Lösung**:

1. **Windows Defender Firewall öffnen**:
   - Windows-Taste → "Firewall" suchen
   - "Windows Defender Firewall"

2. **Eingehende Regel für VcXsrv erstellen**:
   - "Erweiterte Einstellungen"
   - "Eingehende Regeln"
   - "Neue Regel..."
   - "Programm" → Weiter
   - Durchsuchen: `C:\Program Files\VcXsrv\vcxsrv.exe`
   - "Verbindung zulassen"
   - Alle Profile aktivieren
   - Name: "VcXsrv X-Server"

---

### Problem 5: "Error: No module named 'fcntl'"

**Ursache**: Sie haben ServoTester versehentlich lokal auf Windows gestartet

**Lösung**: ServoTester muss auf dem Pi laufen!

```powershell
# RICHTIG: Erst SSH, dann auf dem Pi starten
ssh -X pi@192.168.2.126
cd /home/pi/adeept_raspclaws/Server
python3 ServoTester.py

# FALSCH: Lokal auf Windows
cd C:\...\Server
python ServoTester.py  # ← Funktioniert nicht mit Hardware!
```

---

## Komfort-Verbesserungen

### 1. VcXsrv automatisch starten

**Erstellen Sie eine Batch-Datei**:

```batch
@echo off
REM start_servo_tester.bat
echo Starting VcXsrv X-Server...
start "" "C:\Program Files\VcXsrv\vcxsrv.exe" :0 -multiwindow -clipboard -wgl -ac

timeout /t 2

echo Connecting to Raspberry Pi...
ssh -X pi@192.168.2.126 "cd /home/pi/adeept_raspclaws/Server && python3 ServoTester.py"
```

Speichern als: `start_servo_tester.bat` im Client-Verzeichnis

**Verwendung**:
```powershell
cd C:\Users\SCMJ178\IdeaProjects\Adeept_RaspClaws\Client
.\start_servo_tester.bat
```

---

### 2. SSH-Keys für passwortloses Login

Vermeiden Sie Passwort-Eingaben bei jedem Start:

**Siehe Dokumentation**:
- `Docu/SSH_KEYS_SETUP_DE.md` (Deutsch)
- `Docu/SSH_KEYS_SETUP_EN.md` (Englisch)

---

### 3. VcXsrv Config speichern

**Desktop-Verknüpfung erstellen**:

1. XLaunch öffnen
2. Gewünschte Einstellungen vornehmen
3. Letzte Seite: "Save configuration"
4. Speichern als `servo_tester.xlaunch` auf Desktop
5. Doppelklick auf `servo_tester.xlaunch` → VcXsrv startet mit richtigen Einstellungen

---

## Alternative: MobaXterm

**Einfachere Alternative** (kommerziell, Free-Version verfügbar):

### Vorteile:
- ✅ Integrierter X-Server
- ✅ Keine separate Installation nötig
- ✅ Automatisches X11-Forwarding
- ✅ Tabbed SSH-Sessions

### Installation:

1. Download: https://mobaxterm.mobatek.net/download-home-edition.html
2. "MobaXterm Home Edition" (Free) herunterladen
3. Installer oder Portable Version

### Verwendung:

1. **MobaXterm starten**
2. **Session → SSH**:
   - Remote host: `192.168.2.126`
   - Username: `pi`
   - ☑ X11-Forwarding: Automatisch aktiviert!
3. **OK** → Verbindung wird hergestellt
4. Im Terminal:
   ```bash
   cd /home/pi/adeept_raspclaws/Server
   python3 ServoTester.py
   ```

**Fertig!** X11-Forwarding funktioniert automatisch.

---

## Zusammenfassung

### Einfachste Methode:

```
1. VcXsrv installieren
2. XLaunch starten (Multiple windows, Disable access control)
3. SSH mit X11: ssh -X pi@192.168.2.126
4. Test: xclock
5. ServoTester starten: python3 ServoTester.py
```

### Tägliche Verwendung:

```powershell
# Terminal 1: VcXsrv starten (falls nicht läuft)
# XLaunch oder gespeicherte .xlaunch Datei

# Terminal 2: SSH + ServoTester
ssh -X pi@192.168.2.126
cd /home/pi/adeept_raspclaws/Server
python3 ServoTester.py
```

### Bei Problemen:

1. ✅ VcXsrv läuft? (System Tray Icon)
2. ✅ `echo $DISPLAY` zeigt etwas an?
3. ✅ `xclock` funktioniert?
4. ✅ Firewall-Ausnahme für VcXsrv?

---

## Performance-Tipps

### Für optimale Geschwindigkeit:

1. **Ethernet statt WLAN** nutzen
2. **Kompression aktivieren**: `ssh -X -C`
3. **Trusted X11**: `ssh -Y` (falls `-X` langsam ist)
4. **VcXsrv Einstellungen**:
   - "Disable access control" aktivieren
   - "Native opengl" deaktivieren (stabiler)

### Für niedrige Bandbreite:

```powershell
# SSH mit Kompression und niedrigerer Qualität
ssh -X -C -o "Compression yes" -o "CompressionLevel 9" pi@192.168.2.126
```

---

## Weitere Informationen

**Vollständige SSH-Dokumentation**: `Docu/SSH_KEYS_SETUP_DE.md`

**X11-Forwarding Test-Befehle**:
```bash
# Einfache Tests
xclock          # Uhr anzeigen
xeyes           # Augen anzeigen
xlogo           # X-Logo anzeigen

# Dateibrowser (falls installiert)
pcmanfm         # Raspberry Pi Dateimanager
```

---

**Viel Erfolg mit dem ServoTester über SSH!** 🚀

Bei Fragen: Siehe Troubleshooting-Sektion oder prüfen Sie die Logs mit `echo $DISPLAY`.
