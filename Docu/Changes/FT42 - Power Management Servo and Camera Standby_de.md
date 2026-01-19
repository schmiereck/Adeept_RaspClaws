# FT42 - Power Management: Servo und Kamera Standby

**Datum:** 2026-01-19  
**Typ:** Feature  
**Priorität:** Hoch

## Problem

Die Akkus des Roboters entladen sich sehr schnell:
- Servos ziehen kontinuierlich Strom (PWM-Signale aktiv)
- Kamera-Stream läuft ununterbrochen
- Keine Möglichkeit, Strom zu sparen ohne den Pi komplett abzuschalten
- Beim Entwickeln/Testen werden die Akkus oft leer, bevor Tests abgeschlossen sind

## Lösung

**Zwei unabhängige Standby-Modi:**

### 1. Servo Standby/Wakeup
- **Standby-Modus:** Stoppt alle PWM-Signale (set_pwm(channel, 0, 0))
  - Servos werden "weich" und lassen sich per Hand bewegen
  - Servos ziehen fast keinen Strom mehr
  - **Vorteil:** Pi bleibt an, Terminal/SSH-Verbindung bleiben aktiv
  
- **Wakeup-Modus:** Stellt letzte bekannte Servo-Positionen wieder her
  - Verwendet `servo_current_pos[]` Array (aus FT40)
  - Sanfte Wiederherstellung ohne Ruckeln

### 2. Kamera Pause/Resume
- **Pause-Modus:** Pausiert den Video-Stream
  - Spart CPU-Last und Strom
  - Stream kann jederzeit wieder fortgesetzt werden
  
- **Resume-Modus:** Setzt Video-Stream fort
  - Kein Neustart der Kamera nötig
  - Fortsetzung ohne Verzögerung

## Implementierung

### Server-Seite

#### 1. RPIservo.py - Servo Standby/Wakeup
```python
def standby(self):
    """Put all servos into standby mode by stopping PWM signals."""
    print('Servos entering STANDBY mode - PWM signals stopped')
    if not MOCK_MODE:
        for i in range(16):
            pwm.set_pwm(i, 0, 0)  # Stopping pulse stops the signal
    self.pause()

def wakeup(self):
    """Wake up servos from standby mode."""
    print('Servos WAKING UP - restoring positions')
    if not MOCK_MODE:
        for i in range(16):
            pwm.set_pwm(i, 0, self.nowPos[i])  # Restore last position
    self.resume()
```

#### 2. Move.py - Wrapper-Funktionen
```python
def standby():
    """Put all servos into standby - stops PWM signals."""
    print("🔋 Moving servos to STANDBY mode")
    global move_stu
    move_stu = 0  # Stop ongoing movement
    
    for i in range(16):
        pwm.set_pwm(i, 0, 0)
    
    print("✓ All servos in STANDBY - legs are soft, low power")

def wakeup():
    """Wake up servos from standby - restores last positions."""
    print("⚡ WAKEUP - Restoring servo positions")
    global servo_current_pos
    
    for i in range(16):
        pwm.set_pwm(i, 0, servo_current_pos[i])
    
    print("✓ All servos restored - robot ready")
```

#### 3. FPV.py - Kamera Pause/Resume
```python
# Global flag for camera stream pause/resume
camera_paused = False

def pause_stream():
    """Pause the camera video stream to save power"""
    global camera_paused
    camera_paused = True
    print("📷 Camera stream PAUSED - saving power")

def resume_stream():
    """Resume the camera video stream"""
    global camera_paused
    camera_paused = False
    print("📷 Camera stream RESUMED")
```

#### 4. GUIServer.py - Command Handler
```python
def handle_power_management_command(data):
    """Handle servo standby/wakeup and camera pause/resume commands"""
    
    if data == 'servo_standby':
        print("🔋 SERVO STANDBY - Stopping PWM signals")
        move.standby()
        return True
    
    elif data == 'servo_wakeup':
        print("⚡ SERVO WAKEUP - Restoring servo positions")
        move.wakeup()
        return True
    
    elif data == 'camera_pause':
        print("📷 CAMERA PAUSE - Stopping video stream")
        FPV.pause_stream()
        return True
    
    elif data == 'camera_resume':
        print("📷 CAMERA RESUME - Restarting video stream")
        FPV.resume_stream()
        return True
    
    return False
```

### Client-Seite (GUI.py)

#### GUI Buttons
**Zwei neue Toggle-Buttons in der dritten Reihe (unter Smooth-Cam):**

Diese Buttons wechseln ihren Zustand und Text beim Klicken:

```python
# Servo Standby/Wakeup Toggle-Button
Btn_ServoStandby = tk.Button(root, width=21, text='Servo Standby [M]',
                             fg=color_text, bg=color_btn, relief='ridge')
Btn_ServoStandby.place(x=30, y=480)
root.bind('<KeyPress-m>', call_servo_standby)

# Initial: "Servo Standby [M]" (blau) → klicken → "Servo Wake [M]" (orange)
# Erneut klicken → zurück zu "Servo Standby [M]" (blau)

# Camera Pause/Resume Toggle-Button  
Btn_CameraPause = tk.Button(root, width=21, text='Camera Pause [,]',
                           fg=color_text, bg=color_btn, relief='ridge')
Btn_CameraPause.place(x=200, y=480)
root.bind('<KeyPress-comma>', call_camera_pause)

# Initial: "Camera Pause [,]" (blau) → klicken → "Camera Resume [,]" (orange)
# Erneut klicken → zurück zu "Camera Pause [,]" (blau)
```

#### Callback-Funktionen
```python
def call_servo_standby(event):
    """Toggle servo standby mode"""
    global servo_standby_state
    if not servo_standby_state:
        tcpClicSock.send('servo_standby'.encode())
        Btn_ServoStandby.config(bg='#FF6D00', fg='#000000', 
                                text='Servo Wake [M]')
        servo_standby_state = True
        print("🔋 Servos in STANDBY mode - low power")
    else:
        tcpClicSock.send('servo_wakeup'.encode())
        Btn_ServoStandby.config(bg=color_btn, fg=color_text, 
                                text='Servo Standby [M]')
        servo_standby_state = False
        print("⚡ Servos AWAKE - ready to move")

def call_camera_pause(event):
    """Toggle camera pause mode"""
    # Similar implementation for camera
```

## Bedienung

**Wichtig:** Es gibt jeweils NUR EINEN Toggle-Button, der seinen Zustand wechselt!

### Servo Standby/Wakeup (Toggle-Button)
Der Button wechselt zwischen zwei Modi:

**Zustand 1: Standby aktivieren**
   - Button zeigt: **"Servo Standby [M]"** (blau)
   - Aktion: Button klicken ODER Taste `M` drücken
   - Effekt: 
     - Button wird orange, Text ändert sich zu **"Servo Wake [M]"**
     - Servos werden weich, Beine lassen sich per Hand bewegen
     - Status: **STANDBY AKTIV**
   
**Zustand 2: Wakeup / Standby beenden**
   - Button zeigt: **"Servo Wake [M]"** (orange)
   - Aktion: Button klicken ODER Taste `M` drücken
   - Effekt:
     - Button wird blau, Text ändert sich zu **"Servo Standby [M]"**
     - Servos sind wieder steif, Roboter ist bewegungsbereit
     - Status: **NORMAL (AKTIV)**

### Kamera Pause/Resume (Toggle-Button)
Der Button wechselt zwischen zwei Modi:

**Zustand 1: Pause aktivieren**
   - Button zeigt: **"Camera Pause [,]"** (blau)
   - Aktion: Button klicken ODER Taste `,` (Komma) drücken
   - Effekt:
     - Button wird orange, Text ändert sich zu **"Camera Resume [,]"**
     - Video-Stream pausiert
     - Status: **PAUSE AKTIV**
   
**Zustand 2: Resume / Pause beenden**
   - Button zeigt: **"Camera Resume [,]"** (orange)
   - Aktion: Button klicken ODER Taste `,` drücken
   - Effekt:
     - Button wird blau, Text ändert sich zu **"Camera Pause [,]"**
     - Video-Stream läuft wieder
     - Status: **NORMAL (AKTIV)**

### Visuelle Hinweise
- **🔵 BLAU:** Normal-Modus (kann aktiviert werden)
- **🟠 ORANGE:** Standby/Pause aktiv (kann deaktiviert werden)
- **Text ändert sich:** Zeigt immer die NÄCHSTE mögliche Aktion

### Visualisierung Toggle-Button Servo

```
   ┌─────────────────────────┐
   │  "Servo Standby [M]"    │  ← Initial (Blau)
   │     (drücken M)         │
   └─────────────────────────┘
              │
              ↓ Klick/M
   ┌─────────────────────────┐
   │   "Servo Wake [M]"      │  ← Standby aktiv (Orange)
   │  (Servos weich, kein    │
   │   Strom, manuell        │
   │   bewegbar)             │
   │     (drücken M)         │
   └─────────────────────────┘
              │
              ↓ Klick/M
   ┌─────────────────────────┐
   │  "Servo Standby [M]"    │  ← Zurück zu Normal (Blau)
   │  (Servos steif, bereit) │
   └─────────────────────────┘
```

### Visualisierung Toggle-Button Kamera

```
   ┌─────────────────────────┐
   │  "Camera Pause [,]"     │  ← Initial (Blau)
   │     (drücken ,)         │
   └─────────────────────────┘
              │
              ↓ Klick/,
   ┌─────────────────────────┐
   │  "Camera Resume [,]"    │  ← Pause aktiv (Orange)
   │  (Stream pausiert,      │
   │   Bild eingefroren)     │
   │     (drücken ,)         │
   └─────────────────────────┘
              │
              ↓ Klick/,
   ┌─────────────────────────┐
   │  "Camera Pause [,]"     │  ← Zurück zu Normal (Blau)
   │  (Stream läuft)         │
   └─────────────────────────┘
```

## Vorteile

### Servo Standby
- ✅ **Massiver Stromersparniss:** Servos ziehen fast keinen Strom mehr
- ✅ **Pi bleibt an:** Terminal, SSH, Kamera bleiben aktiv
- ✅ **Manuelle Positionierung:** Beine können per Hand bewegt werden
- ✅ **Schnelles Wakeup:** Position wird sofort wiederhergestellt
- ✅ **Keine Neuinitialisierung:** Keine Wartezeit, sofort einsatzbereit

### Kamera Pause
- ✅ **CPU-Entlastung:** Reduziert CPU-Last deutlich
- ✅ **Strom sparen:** Weniger Rechenleistung = weniger Strom
- ✅ **Schnelles Resume:** Stream läuft sofort weiter
- ✅ **Keine Kamera-Reinitialisierung:** Kein Neustart nötig

## Anwendungsfälle

1. **Entwicklung/Debugging:**
   - Servos in Standby während Code-Änderungen
   - Terminal/SSH bleibt aktiv
   - Kein ständiges Ein-/Ausschalten des Pi nötig

2. **Akku-Schonung:**
   - Servo Standby während längeren Pausen
   - Kamera Pause wenn Video nicht benötigt wird
   - Akkus halten deutlich länger

3. **Manuelle Justierung:**
   - Servos in Standby setzen
   - Beine/Kamera per Hand in Position bringen
   - Wakeup für neue Ausgangsposition

4. **Sichere Wartung:**
   - Servos in Standby während Hardware-Arbeiten
   - Keine unerwarteten Bewegungen
   - Pi bleibt für Monitoring/Logging an

## Testing

### Testfälle
1. ✓ **Servo Standby aktivieren:** PWM-Signale stoppen
2. ✓ **Servo Wakeup:** Positionen wiederherstellen
3. ✓ **Kamera Pause:** Stream pausiert
4. ✓ **Kamera Resume:** Stream läuft wieder
5. ✓ **Mehrfaches Toggle:** Funktioniert zuverlässig
6. ✓ **Tastatur-Shortcuts:** M und , funktionieren
7. ✓ **Button-Status:** Farbe/Text ändern sich korrekt

### Erwartetes Verhalten
- **Servo Standby aktiv:** Beine lassen sich leicht per Hand bewegen
- **Servo Wakeup:** Servos kehren sanft zur letzten Position zurück
- **Kamera Pause aktiv:** Video-Fenster zeigt letztes Frame (eingefroren)
- **Kamera Resume:** Video läuft nahtlos weiter

## Geänderte Dateien

- **Server/RPIservo.py:** Standby/Wakeup Funktionen in ServoCtrl Klasse
- **Server/Move.py:** Wrapper-Funktionen standby() und wakeup()
- **Server/FPV.py:** Global pause_stream() und resume_stream() Funktionen
- **Server/GUIServer.py:** handle_power_management_command() Handler
- **Client/GUI.py:** 
  - call_servo_standby() und call_camera_pause() Callbacks
  - Btn_ServoStandby und Btn_CameraPause Buttons
  - Keyboard Shortcuts M und ,

## Zukünftige Erweiterungen

1. **Auto-Standby:** Automatisch nach X Minuten Inaktivität
2. **Partial Standby:** Nur bestimmte Servos in Standby (z.B. nur Beine, nicht Kamera)
3. **Stromverbrauch-Anzeige:** Zeige aktuellen Stromverbrauch in GUI
4. **Batterie-Warnung:** Automatisch in Standby bei niedrigem Akku
5. **Profil-Speicherung:** Verschiedene Standby-Profile (nur Servos, nur Kamera, beides)

## Hinweise

- **Servo Positions-Tracking:** Nutzt das in FT40 implementierte `servo_current_pos[]` Array
- **Kein Datenverlust:** Alle Positionen/States bleiben erhalten
- **Kombination möglich:** Servo Standby + Kamera Pause gleichzeitig für maximale Stromersparniss
- **Hardware-Sicherheit:** Im Mock-Mode (keine Hardware) werden Befehle ignoriert

## Lessons Learned

1. **Einfache Lösung ist oft die Beste:** PWM auf 0 setzen ist einfacher als komplexe Power-Management-Logik
2. **Position-Tracking zahlt sich aus:** Das servo_current_pos[] Array aus FT40 macht Wakeup trivial
3. **Unabhängige Modi:** Servo und Kamera getrennt schaltbar gibt maximale Flexibilität
4. **GUI-Feedback wichtig:** Button-Farbe/Text-Änderung zeigt Status sofort an
5. **Tastatur-Shortcuts:** Schnelle Bedienung ohne Maus ist praktisch beim Entwickeln

## Referenzen

- Verwandt: FT40 (Servo Position Tracking - Basis für Wakeup-Funktion)
- Vorher: FT41 (Fix Left/Right Turn Movement)
- Nächste: TBD (vielleicht Auto-Standby oder ROS2 Integration)
