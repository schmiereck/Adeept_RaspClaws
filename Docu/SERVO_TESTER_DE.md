# Servo Tester Tool - Dokumentation

## Übersicht

Das Servo Tester Tool ist eine eigenständige GUI-Anwendung zum Testen und Kalibrieren der einzelnen Servos des RaspClaws Roboters.

**Datei**: `Server/ServoTester.py`

**Features**:
- ✅ Einzelne Steuerung aller 8 Servo-Kanäle
- ✅ Echtzeit-PWM-Wert-Anzeige
- ✅ Schieberegler für präzise Positionierung
- ✅ Schnell-Buttons (Reset, Min, Max)
- ✅ Mock-Mode für Entwicklung ohne Hardware
- ✅ SSH-fähig (X11-Forwarding)

---

## Installation & Start

### 1. Auf dem Raspberry Pi (mit Hardware)

**Via SSH mit X11-Forwarding**:

```bash
# Von Windows aus (mit X-Server wie VcXsrv oder Xming)
ssh -X pi@192.168.2.126

# Auf dem Pi
cd /home/pi/adeept_raspclaws/Server
python3 ServoTester.py
```

**Direkt auf dem Pi** (mit Monitor):

```bash
cd /home/pi/adeept_raspclaws/Server
python3 ServoTester.py
```

### 2. Auf Windows (Mock-Mode)

Für Entwicklung und Testing ohne Hardware:

```powershell
cd C:\Users\SCMJ178\IdeaProjects\Adeept_RaspClaws\Server
python ServoTester.py
```

**Hinweis**: Im Mock-Mode werden die PWM-Werte nur simuliert und in der Konsole ausgegeben.

---

## GUI-Übersicht

### Layout

```
┌─────────────────────────────────────────┐
│          Servo Tester                   │
│         [HARDWARE MODE]                 │
│   Move sliders to test each servo       │
├─────────────────────────────────────────┤
│                                         │
│  Head Up/Down (Ch 0)            [300]   │
│  [================|==============]      │
│  Min: 100              Max: 600         │
│                                         │
│  Head Left/Right (Ch 1)         [300]   │
│  [================|==============]      │
│  Min: 100              Max: 600         │
│                                         │
│  Leg 1 (Ch 4)                   [300]   │
│  [================|==============]      │
│  Min: 100              Max: 600         │
│                                         │
│  ... (weitere Servos) ...               │
│                                         │
├─────────────────────────────────────────┤
│ [Reset All]  [All to Min]  [All to Max] │
│                              [Quit]     │
└─────────────────────────────────────────┘
```

### Komponenten

#### 1. **Servo-Schieberegler**
- **Name**: Servo-Bezeichnung (z.B. "Head Up/Down")
- **Kanal**: PCA9685 Kanal-Nummer (Ch 0-9)
- **Aktueller Wert**: Grüne Zahl rechts (aktueller PWM-Wert)
- **Slider**: Horizontal, Bereich 100-600
- **Min/Max**: Anzeige des gültigen Bereichs

#### 2. **Kontroll-Buttons**

**Reset All to Default**:
- Setzt alle Servos auf Position 300 (Mittelstellung)
- Farbe: Blau
- Empfohlen beim Start/Ende

**All to Min**:
- Setzt alle Servos auf minimale Position (100)
- Farbe: Orange
- Vorsicht: Kann mechanische Grenzen erreichen!

**All to Max**:
- Setzt alle Servos auf maximale Position (600)
- Farbe: Orange
- Vorsicht: Kann mechanische Grenzen erreichen!

**Quit**:
- Beendet die Anwendung
- Setzt vorher alle Servos auf Default
- Farbe: Rot

---

## Servo-Kanäle

### Kanal-Mapping & Hardware-Layout

Die GUI spiegelt das physische Layout des Roboters wider mit **zwei Spalten** (Links/Rechts).

**Jedes Bein hat 2 Servos**:
- **Rotation**: Horizontale Bewegung (vor/zurück)
- **Height**: Vertikale Bewegung (hoch/runter)

#### Linke Seite (Kanäle 0-5)

| Bein               | Position    | Servo-Funktion | Kanal |
|--------------------|-------------|----------------|-------|
| **Front-Left**     | Vorne-Links | Rotation       | 0     |
| **(Left I)**       |             | Height         | 1     |
| **Back-Left**      | Hinten-Links| Rotation       | 2     |
| **(Left II)**      |             | Height         | 3     |
| **Left III**       | ?           | Rotation       | 4     |
|                    |             | Height         | 5     |

#### Rechte Seite (Kanäle 6-11)

| Bein               | Position     | Servo-Funktion | Kanal |
|--------------------|--------------|----------------|-------|
| **Front-Right**    | Vorne-Rechts | Rotation       | 6     |
| **(Right I)**      |              | Height         | 7     |
| **Back-Right**     | Hinten-Rechts| Rotation       | 8     |
| **(Right II)**     |              | Height         | 9     |
| **Right III**      | ?            | Rotation       | 10    |
|                    |              | Height         | 11    |

#### Kopf (Kanäle 12-13)

| Komponente | Servo-Funktion | Kanal |
|------------|----------------|-------|
| **Head**   | Up/Down        | 12    |
|            | Left/Right     | 13    |

### GUI-Layout

```
┌────────────────────────────────────────────────────┐
│          Servo Tester - Hardware Layout            │
│               [HARDWARE MODE]                      │
│   Layout reflects robot hardware: Left | Right     │
├────────────────────┬───────────────────────────────┤
│    LEFT SIDE       │       RIGHT SIDE              │
├────────────────────┼───────────────────────────────┤
│ Front-Left (I)     │ Front-Right (I)               │
│  ├─ Rotation [300] │  ├─ Rotation [300]            │
│  └─ Height   [300] │  └─ Height   [300]            │
│                    │                               │
│ Back-Left (II)     │ Back-Right (II)               │
│  ├─ Rotation [300] │  ├─ Rotation [300]            │
│  └─ Height   [300] │  └─ Height   [300]            │
│                    │                               │
│ Left III           │ Right III                     │
│  ├─ Rotation [300] │  ├─ Rotation [300]            │
│  └─ Height   [300] │  └─ Height   [300]            │
└────────────────────┴───────────────────────────────┘
                HEAD SERVOS
        ┌──────────────────────┐
        │ Up/Down      [300]   │
        │ Left/Right   [300]   │
        └──────────────────────┘
     [Reset] [Min] [Max]       [Quit]
```

### PWM-Werte

- **Minimum**: 100 (eine Endposition)
- **Maximum**: 600 (andere Endposition)
- **Default**: 300 (Mittelstellung)
- **Frequenz**: 50 Hz (Standard für Servos)

**Wichtig**: Die tatsächlichen mechanischen Grenzen können von Servo zu Servo variieren. Testen Sie vorsichtig!

---

## Verwendungs-Szenarien

### 1. Servo-Test nach Montage

Nach dem Zusammenbau oder Tausch eines Servos:

1. ✅ Starte ServoTester
2. ✅ Klicke "Reset All to Default"
3. ✅ Bewege jeden Slider einzeln langsam von Min zu Max
4. ✅ Prüfe, ob Servo reagiert und sich in die richtige Richtung bewegt
5. ✅ Notiere mechanische Grenzen (falls vor 100 oder 600)

### 2. Kalibrierung

Finde optimale PWM-Werte für Neutral-Positionen:

1. ✅ Bewege Slider zu gewünschter Position
2. ✅ Notiere PWM-Wert aus grüner Anzeige
3. ✅ Übertrage Werte in `Move.py` oder `RPIservo.py`

### 3. Bewegungsbereich-Test

Teste maximale Bewegungsbereiche:

1. ✅ Setze Servo auf Min (100)
2. ✅ Prüfe, ob mechanische Grenze erreicht (Geräusch/Widerstand)
3. ✅ Falls ja: Erhöhe Minimum in Schritten von 10
4. ✅ Wiederhole für Maximum
5. ✅ Aktualisiere Konstanten im Code

### 4. Fehlerdiagnose

Bei Problemen mit einem Servo:

1. ✅ Teste Servo einzeln mit ServoTester
2. ✅ Kein Bewegung → Hardware-Problem (Verkabelung/Servo defekt)
3. ✅ Zucken/Rauschen → PWM-Problem oder Stromversorgung
4. ✅ Falsche Richtung → Mechanisch falsch montiert

---

## SSH X11-Forwarding Setup

### Windows mit VcXsrv

1. **VcXsrv installieren**:
   - Download: https://sourceforge.net/projects/vcxsrv/
   - Installieren mit Standardeinstellungen

2. **XLaunch starten**:
   - Multiple windows
   - Display number: 0
   - Start no client
   - ✅ Clipboard aktivieren
   - ✅ Disable access control aktivieren

3. **SSH Verbindung mit X11**:
   ```powershell
   ssh -X pi@192.168.2.126
   ```

4. **ServoTester starten**:
   ```bash
   cd /home/pi/adeept_raspclaws/Server
   python3 ServoTester.py
   ```

### Problembehandlung X11

**"Cannot connect to X server"**:
```bash
# DISPLAY Variable prüfen
echo $DISPLAY
# Sollte sein: localhost:10.0 oder ähnlich

# Manuell setzen falls nötig
export DISPLAY=localhost:10.0
```

**"X11 forwarding request failed"**:
```bash
# X11 Forwarding auf Pi aktivieren
sudo nano /etc/ssh/sshd_config
# Zeile ändern: X11Forwarding yes
sudo systemctl restart ssh
```

---

## Code-Architektur

### Klassen-Struktur

```python
# ServoPWM
- Wrapper für PCA9685
- Mock-Mode Unterstützung
- set_pwm(channel, on, off)
- get_current_value(channel)

# ServoTesterGUI
- Hauptanwendung
- create_widgets()           # GUI Aufbau
- create_servo_control()     # Einzelner Servo-Regler
- on_slider_change()         # Slider Event Handler
- reset_all_servos()         # Reset Funktion
- set_all_min/max()          # Min/Max Funktionen
```

### Konfiguration

Anpassbare Konstanten am Dateianfang:

```python
# Servo-Kanäle hinzufügen/ändern
SERVO_CHANNELS = {
    'New Servo': 10,  # Neuer Kanal
}

# PWM-Bereiche anpassen
PWM_MIN = 100
PWM_MAX = 600
PWM_DEFAULT = 300

# Hardware-Adresse ändern
PCA9685_ADDRESS = 0x40
```

---

## Erweiterungsmöglichkeiten

### Geplante Features (Optional)

- [ ] **Preset-Positionen**: Speichern/Laden von Servo-Konfigurationen
- [ ] **Sequenz-Recorder**: Bewegungsabläufe aufzeichnen und abspielen
- [ ] **Individuelle Min/Max**: Pro Servo unterschiedliche Bereiche
- [ ] **Speed-Control**: Geschwindigkeit der Servo-Bewegung einstellen
- [ ] **Keyboard-Shortcuts**: Schnelle Steuerung per Tastatur
- [ ] **Config-File**: Externe Konfigurationsdatei (JSON/YAML)

### Implementierungs-Beispiele

**Preset-Positionen speichern**:
```python
import json

def save_preset(name):
    preset = {ch: slider.get() for ch, slider in self.sliders.items()}
    with open(f'presets/{name}.json', 'w') as f:
        json.dump(preset, f)

def load_preset(name):
    with open(f'presets/{name}.json', 'r') as f:
        preset = json.load(f)
    for channel, value in preset.items():
        self.sliders[int(channel)].set(value)
```

---

## Sicherheitshinweise

### ⚠️ Wichtige Warnungen

1. **Mechanische Grenzen**: 
   - Servos können Kraft ausüben!
   - Bei Widerstand SOFORT stoppen
   - Min/Max vorsichtig testen

2. **Stromversorgung**:
   - Servos benötigen externe Stromversorgung
   - Nicht nur über USB!
   - Batterie muss geladen sein

3. **Verkabelung**:
   - Vor Test prüfen: Alle Servos korrekt angeschlossen?
   - PCA9685 korrekt verbunden (SDA/SCL)?
   - Keine losen Kabel

4. **Erste Inbetriebnahme**:
   - Immer mit "Reset All" starten
   - Kleine Schritte bei neuen Servos
   - Roboter auf sichere Unterlage

---

## Troubleshooting

### Problem: "Could not initialize PCA9685"

**Ursachen**:
- PCA9685 nicht angeschlossen
- Falsche I²C Adresse
- I²C nicht aktiviert

**Lösung**:
```bash
# I²C aktivieren
sudo raspi-config
# → Interface Options → I2C → Enable

# I²C Geräte scannen
sudo i2cdetect -y 1
# Sollte 0x40 zeigen

# Falls andere Adresse, in Code anpassen:
PCA9685_ADDRESS = 0x5F  # Oder was auch immer angezeigt wird
```

### Problem: GUI startet nicht via SSH

**Ursachen**:
- X11-Forwarding nicht aktiviert
- DISPLAY Variable fehlt
- Firewall blockiert X11

**Lösung**:
```bash
# X11-Forwarding testen
ssh -X pi@192.168.2.126
echo $DISPLAY  # Sollte etwas ausgeben

# Einfache X11 Test
xclock  # Sollte eine Uhr anzeigen

# Falls Fehler, siehe "SSH X11-Forwarding Setup"
```

### Problem: Servo bewegt sich nicht

**Checkliste**:
- [ ] PCA9685 korrekt initialisiert? (nicht MOCK MODE)
- [ ] Servo am richtigen Kanal angeschlossen?
- [ ] Externe Stromversorgung angeschlossen?
- [ ] Batterie geladen?
- [ ] Servo defekt? (mit anderem Kanal testen)

---

## Integration mit Haupt-Projekt

### Verwendung in Move.py

Optimale PWM-Werte aus ServoTester übernehmen:

```python
# In Move.py oder RPIservo.py
HEAD_VERTICAL_MIN = 150    # Aus ServoTester ermittelt
HEAD_VERTICAL_MAX = 550
HEAD_VERTICAL_DEFAULT = 300
```

### Kalibrierungs-Workflow

1. ✅ ServoTester öffnen
2. ✅ Optimale Werte für jeden Servo finden
3. ✅ Werte notieren
4. ✅ In `Move.py` übertragen
5. ✅ Haupt-Anwendung neu starten
6. ✅ Bewegungen testen

---

## Zusammenfassung

**ServoTester.py** ist ein essentielles Tool für:
- ✅ Hardware-Tests nach Montage
- ✅ Servo-Kalibrierung
- ✅ Fehlerdiagnose
- ✅ Bewegungsbereich-Ermittlung

**Vorteile**:
- 🎯 Einfache Bedienung
- 🔧 Keine Code-Änderungen nötig
- 🖥️ SSH-kompatibel
- 🛡️ Mock-Mode für Entwicklung
- 📊 Echtzeit-Feedback

**Empfehlung**: Als erstes Tool nach Hardware-Setup verwenden!

---

**Autor**: GitHub Copilot  
**Datum**: 2026-01-18  
**Version**: 1.0  
**Status**: ✅ Produktionsbereit
