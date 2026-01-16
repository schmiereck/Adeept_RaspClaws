# PCA9685 Hardware-Fehler beheben

## Problem
```
OSError: [Errno 5] Input/output error
```

Beim Starten des Servers auf dem Raspberry Pi tritt ein I2C-Fehler auf, weil der **PCA9685 Servo-Controller** nicht erreichbar ist.

---

## Ursachen

1. **I2C nicht aktiviert** auf dem Raspberry Pi
2. **PCA9685 nicht angeschlossen** oder lose Verbindung
3. **Falsche I2C-Adresse** (Standard ist `0x40`, manche Boards nutzen `0x5F`)
4. **Hardware-Defekt** am PCA9685

---

## Lösungen

### **Lösung 1: I2C aktivieren (empfohlen für Hardware-Betrieb)**

**Auf dem Raspberry Pi:**

```bash
sudo raspi-config
```

→ **Interface Options** → **I2C** → **Enable**

**Neu starten:**
```bash
sudo reboot
```

**Prüfen, ob PCA9685 erkannt wird:**
```bash
sudo i2cdetect -y 1
```

**Erwartete Ausgabe:**
```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
40: 40 -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
```

Falls `40` erscheint → Hardware ist OK (Standard-Adresse)  
Falls `5f` erscheint → Hardware nutzt alternative Adresse  
Falls nichts erscheint → Hardware-Problem

---

### **Lösung 2: Mock-Modus (für Tests ohne Hardware)**

**Was wurde geändert:**

Die Dateien `RPIservo.py` und `GUIServer.py` wurden angepasst, um auch **ohne PCA9685-Hardware** zu starten.

**Vorher:**
```python
pwm = Adafruit_PCA9685.PCA9685(address=0x5F, busnum=1)  # ❌ Crash bei Fehler
```

**Jetzt:**
```python
try:
    pwm = Adafruit_PCA9685.PCA9685(address=0x40, busnum=1)  # Geändert auf 0x40 (Standard-Adresse)
    print("PCA9685 initialized successfully")
except (OSError, IOError) as e:
    print(f"Warning: Could not initialize PCA9685: {e}")
    print("Running in MOCK MODE - servo commands will be ignored")
    class MockPWM:
        def set_pwm(self, channel, on, off):
            pass
    pwm = MockPWM()  # ✅ Mock-Objekt statt Crash
```

**Vorteile:**
- ✅ Server startet auch ohne Hardware
- ✅ Netzwerk-Tests möglich
- ✅ GUI-Entwicklung ohne Roboter möglich
- ⚠️ Servo-Befehle werden ignoriert (Mock-Modus)

---

## Geänderte Dateien

### 1. **Server/RPIservo.py**
- Try-Except-Block um PCA9685-Initialisierung
- MockPWM-Klasse für Hardware-losen Betrieb

### 2. **Server/GUIServer.py**
- Try-Except-Block um PCA9685-Initialisierung
- MockPWM-Klasse für Hardware-losen Betrieb

---

## Deployment

### **Änderungen auf GitHub pushen:**
```bash
git add Server/RPIservo.py Server/GUIServer.py Docu/PCA9685_FIX.md
git commit -m "Fix: Add mock mode for PCA9685 when hardware not available"
git push
```

### **Auf dem Raspberry Pi:**
```bash
cd /home/pi/adeept_raspclaws
git pull
```

### **Server neu starten:**
```bash
sudo python3 /home/pi/adeept_raspclaws/Server/server.py
```

---

## Erwartete Ausgabe

### **Mit Hardware:**
```
PCA9685 initialized successfully on address 0x40
PCA9685 initialized in GUIServer on address 0x40
```

### **Ohne Hardware (Mock-Modus):**
```
Warning: Could not initialize PCA9685: [Errno 5] Input/output error
Running in MOCK MODE - servo commands will be ignored
Warning: Could not initialize PCA9685 in GUIServer: [Errno 5] Input/output error
Running in MOCK MODE - servo commands will be ignored
```

→ Server läuft trotzdem weiter! ✅

---

## Nächste Schritte

1. **Für Produktion:** I2C aktivieren und Hardware anschließen (Lösung 1)
2. **Für Tests:** Mock-Modus verwenden (automatisch aktiv bei Fehler)
3. **Hardware prüfen:** `sudo i2cdetect -y 1` ausführen

---

## Troubleshooting

### **Problem: I2C ist aktiviert, aber `5f` erscheint nicht**
- Verkabelung prüfen (SDA, SCL, VCC, GND)
- PCA9685 auf anderen Pi testen
- Andere I2C-Adresse probieren (`0x40` statt `0x5F`)

### **Problem: Mock-Modus ist aktiv, aber ich will Hardware nutzen**
- I2C aktivieren (siehe Lösung 1)
- Raspberry Pi neu starten
- Hardware-Verbindung prüfen
