# WS2812 LED-Problem Diagnose und Behebung

## Problem
Die WS2812 RGB-LEDs (vorne und oben am Roboter) leuchten nicht bzw. bleiben dunkel.

---

## ✅ Implementierte Code-Verbesserungen

### **1. Bessere Fehlerbehandlung in GUIServer.py**

**Vorher:**
```python
try:
    ws2812 = robotLight.Adeept_SPI_LedPixel(16, 255)
    ws2812.start()
    ws2812.breath(70,70,255)
except:
    pass  # ❌ Fehler werden verschluckt
```

**Jetzt:**
```python
ws2812 = None
try:
    print("Initializing WS2812 LEDs...")
    ws2812 = robotLight.Adeept_SPI_LedPixel(16, 255)
    if ws2812.check_spi_state() != 0:
        print("WS2812 initialized successfully")
        ws2812.start()
        ws2812.breath(70, 70, 255)
    else:
        print("Warning: SPI not available for WS2812 LEDs")
        ws2812 = None
except Exception as e:
    print(f"Warning: Could not initialize WS2812 LEDs: {e}")
    ws2812 = None
```

**Vorteile:**
- ✅ Fehlermeldungen werden angezeigt
- ✅ Server startet auch ohne funktionierende LEDs
- ✅ Klare Diagnose möglich

### **2. Sicherheitsprüfungen in run()-Funktion**

**Vorher:**
```python
elif 'police' == data:
    ws2812.police()  # ❌ Crash wenn ws2812 = None
```

**Jetzt:**
```python
elif 'police' == data:
    if ws2812:
        ws2812.police()  # ✅ Nur ausführen wenn verfügbar
    tcpCliSock.send(('police').encode())
```

---

## 🔍 Fehlerdiagnose auf dem Raspberry Pi

### **Schritt 1: Server-Log prüfen**

Nach dem Pullen und Server-Neustart:

```bash
cd /home/pi/adeept_raspclaws
git pull
sudo python3 Server/server.py
```

**Suche nach folgenden Meldungen:**

#### **Fall A: LEDs funktionieren ✅**
```
Initializing WS2812 LEDs...
WS2812 initialized successfully
WS2812 LEDs set to blue (connected state)
```
→ Software ist OK, wenn LEDs trotzdem dunkel sind → **Hardware-Problem**

#### **Fall B: SPI nicht verfügbar ⚠️**
```
Initializing WS2812 LEDs...
Warning: SPI not available for WS2812 LEDs
```
→ **SPI ist nicht aktiviert** (siehe Lösung unten)

#### **Fall C: Fehler beim Initialisieren ❌**
```
Initializing WS2812 LEDs...
Warning: Could not initialize WS2812 LEDs: [Errno XY] ...
```
→ **Hardware- oder Konfigurationsproblem** (siehe unten)

---

## 🔧 Lösungen

### **Lösung 1: SPI aktivieren**

**Auf dem Raspberry Pi:**

```bash
sudo raspi-config
```

→ **Interface Options** → **SPI** → **Enable**

```bash
sudo reboot
```

**Nach dem Neustart prüfen:**

```bash
ls -l /dev/spidev*
```

**Erwartete Ausgabe:**
```
crw-rw---- 1 root spi 153, 0 Jan 16 10:00 /dev/spidev0.0
crw-rw---- 1 root spi 153, 1 Jan 16 10:00 /dev/spidev0.1
```

Falls `/dev/spidev0.0` existiert → SPI ist aktiviert ✅

---

### **Lösung 2: Config.txt prüfen**

**Datei öffnen:**

```bash
sudo nano /boot/firmware/config.txt
```

**Stelle sicher, dass folgende Zeile NICHT auskommentiert ist:**

```
dtparam=spi=on
```

Falls die Zeile fehlt oder mit `#` auskommentiert ist, aktiviere sie:

```
dtparam=spi=on
```

**Speichern:** `Ctrl+O`, `Enter`, `Ctrl+X`

**Neustart:**
```bash
sudo reboot
```

---

### **Lösung 3: WS2812 Verkabelung prüfen**

**Die WS2812 LEDs benötigen:**

| Pin | Funktion | GPIO |
|-----|----------|------|
| VCC | 5V | 5V |
| GND | Masse | GND |
| DIN | Daten | GPIO10 (SPI0-MOSI) |

**Prüfen:**
1. Sind alle Kabel fest verbunden?
2. Ist die 5V-Versorgung angeschlossen?
3. Ist GPIO10 (Pin 19) mit dem DIN-Pin verbunden?

**Schaltplan:**
```
Raspberry Pi          WS2812 LED Strip
Pin 19 (GPIO10) ----> DIN
Pin 2  (5V)     ----> VCC/5V
Pin 6  (GND)    ----> GND
```

---

### **Lösung 4: Manuelle LED-Tests**

**Test-Skript erstellen:**

```bash
cd /home/pi/adeept_raspclaws
nano test_leds.py
```

**Inhalt:**
```python
#!/usr/bin/env python3
import sys
sys.path.append('/home/pi/adeept_raspclaws/Server')
import RobotLight as robotLight
import time

print("Testing WS2812 LEDs...")

try:
    ws2812 = robotLight.Adeept_SPI_LedPixel(16, 255)
    
    if ws2812.check_spi_state() != 0:
        print("SPI initialized successfully")
        ws2812.start()
        
        print("Setting LEDs to RED...")
        ws2812.set_all_led_color_data(255, 0, 0)
        ws2812.show()
        time.sleep(2)
        
        print("Setting LEDs to GREEN...")
        ws2812.set_all_led_color_data(0, 255, 0)
        ws2812.show()
        time.sleep(2)
        
        print("Setting LEDs to BLUE...")
        ws2812.set_all_led_color_data(0, 0, 255)
        ws2812.show()
        time.sleep(2)
        
        print("Turning LEDs OFF...")
        ws2812.set_all_led_color_data(0, 0, 0)
        ws2812.show()
        
        ws2812.led_close()
        print("Test completed successfully!")
    else:
        print("ERROR: SPI not available")
except Exception as e:
    print(f"ERROR: {e}")
```

**Ausführen:**
```bash
sudo python3 test_leds.py
```

**Erwartetes Ergebnis:**
- LEDs leuchten **ROT** (2 Sekunden)
- LEDs leuchten **GRÜN** (2 Sekunden)
- LEDs leuchten **BLAU** (2 Sekunden)
- LEDs gehen **AUS**

Falls das funktioniert → **Hardware ist OK, Software-Problem in GUIServer.py**  
Falls das nicht funktioniert → **Hardware- oder Verkabelungsproblem**

---

## 📋 Deployment

### **Änderungen auf GitHub pushen:**

```powershell
git add Server/GUIServer.py Docu/LED_PROBLEM.md
git commit -m "Fix: Improve WS2812 LED error handling and diagnostics"
git push
```

### **Auf dem Raspberry Pi:**

```bash
cd /home/pi/adeept_raspclaws
git pull
sudo python3 Server/server.py
```

**Achte auf die Ausgabe:**
```
Initializing WS2812 LEDs...
WS2812 initialized successfully
```

---

## 🎯 Erwartetes Verhalten nach dem Fix

### **Beim Server-Start:**
1. Blaues Atmen (breath-Effekt)
2. Bei Verbindung: Blau leuchtend

### **Bei Befehlen:**
- **Police-Button:** Rot/Blau blinkend
- **Police Off:** Zurück zu Blau

---

## ❓ Troubleshooting

### **Problem: "Permission denied" beim Zugriff auf SPI**

**Lösung:** Benutzer zur `spi`-Gruppe hinzufügen:

```bash
sudo usermod -a -G spi pi
```

Neu anmelden oder neu starten.

---

### **Problem: LEDs zeigen falsche Farben**

**Mögliche Ursache:** Falsche LED-Typ-Konfiguration

**In GUIServer.py Zeile 286 ändern:**

```python
# Standard ist GRB:
ws2812 = robotLight.Adeept_SPI_LedPixel(16, 255)

# Falls Farben falsch sind, versuche RGB:
ws2812 = robotLight.Adeept_SPI_LedPixel(16, 255, sequence='RGB')

# Oder andere Varianten: 'RBG', 'GBR', 'BRG', 'BGR'
```

---

### **Problem: Nur ein Teil der LEDs leuchtet**

**Mögliche Ursache:** Defekte LED im Strip oder falsche Anzahl

**In GUIServer.py Zeile 286:**

```python
ws2812 = robotLight.Adeept_SPI_LedPixel(16, 255)  # 16 LEDs
```

Falls dein Roboter eine andere Anzahl hat, ändere die Zahl entsprechend.

---

## ✅ Zusammenfassung

Nach den Code-Änderungen:
1. ✅ Server startet auch ohne funktionierende LEDs
2. ✅ Fehlermeldungen zeigen das Problem an
3. ✅ Keine Crashes mehr durch fehlende LEDs
4. ✅ Einfache Diagnose möglich

**Nächster Schritt:** Pull auf dem Pi und Log-Ausgaben prüfen!
