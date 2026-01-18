# Batterie-Überwachungs-Feature

## Übersicht

Das Batterie-Überwachungs-Feature zeigt die aktuelle Batteriespannung und den Ladezustand in Prozent mit farbcodierter visueller Rückmeldung in der GUI an.

## Hardware

**ADC Chip**: ADS7830 (I²C Adresse 0x48)
- 8-Kanal 8-Bit ADC
- Verbindung über I²C Bus

**Spannungsteiler**:
- R15 = 3000Ω
- R17 = 1000Ω
- Teilerverhältnis = 0.25

**Batterie-Spezifikationen**:
- Referenzspannung (Voll): 8.4V
- Warn-Schwellwert: 6.0V
- Spannungsbereich: 6.0V - 8.4V

## Implementierung

### Server-Seite (GUIServer.py)

1. **ADS7830 Initialisierung**:
   - Versucht ADS7830 über I²C zu initialisieren
   - Fällt zurück auf Mock-Modus (liefert 0.0V), wenn Hardware nicht verfügbar
   - Kein Absturz, wenn Batterie-Überwachung nicht verfügbar

2. **Spannungs-Messung**:
   - Funktion: `get_battery_voltage()`
   - Liest ADC Kanal 0
   - Wandelt ADC-Wert in tatsächliche Spannung um (Spannungsteiler-Verhältnis)
   - Gibt Spannung als String zurück (2 Dezimalstellen)

3. **INFO-Protokoll Erweiterung**:
   - Altes Format: `INFO:<CPU_TEMP> <CPU_USE> <RAM_USE>`
   - Neues Format: `INFO:<CPU_TEMP> <CPU_USE> <RAM_USE> <BATTERY_VOLTAGE>`
   - Wird jede Sekunde an den Client gesendet

### Client-Seite (GUI.py)

1. **Neues GUI Element**:
   - Label: `BATTERY_lab`
   - Position: Unter RAM Usage (x=400, y=105)
   - Breite: 18 Zeichen

2. **Anzeige-Format**:
   - "Battery: N/A" - wenn Spannung = 0.0V (Überwachung nicht verfügbar)
   - "Battery: 7.5V (62%)" - mit farbcodiertem Hintergrund

3. **Farb-Codierung**:
   - **Grün (#4CAF50)**: ≥ 60% Ladung (≥ 7.44V)
   - **Orange (#FF9800)**: 30-60% Ladung (6.72V - 7.44V)
   - **Rot (#F44336)**: < 30% Ladung (< 6.72V)
   - **Grau (#757575)**: Batterie-Überwachung nicht verfügbar

4. **Prozent-Berechnung**:
   ```python
   battery_percent = ((voltage - 6.0) / (8.4 - 6.0)) * 100
   ```
   - 6.0V = 0%
   - 8.4V = 100%
   - Begrenzt auf 0-100% Bereich

## Mock-Modus

Falls ADS7830 nicht verfügbar ist (Entwicklungsumgebung ohne Hardware):
- Server gibt Warnung aus: "⚠ Battery monitoring not available"
- Funktion gibt "0.0" Spannung zurück
- GUI zeigt "Battery: N/A" mit grauem Hintergrund
- Keine Fehler oder Abstürze

## Testen

### Ohne Hardware (Mock-Modus)
```bash
# Server Ausgabe
⚠ Battery monitoring not available: [Errno 2] No such file or directory

# GUI zeigt
Battery: N/A
```

### Mit Hardware
```bash
# Server Ausgabe
✓ ADS7830 battery monitor initialized successfully

# GUI zeigt (Beispiele)
Battery: 8.2V (96%)   # Grüner Hintergrund
Battery: 7.0V (42%)   # Oranger Hintergrund
Battery: 6.2V (8%)    # Roter Hintergrund
```

## Rückwärtskompatibilität

Die Implementierung ist rückwärtskompatibel:
- Alte Server (ohne Batterie) senden 3 Werte → GUI zeigt "Battery: N/A"
- Neue Server senden 4 Werte → GUI zeigt Batteriespannung und Prozent
- Keine Fehler bei Protokoll-Mismatch

## Abhängigkeiten

**Server**:
- `smbus` - für I²C Kommunikation
- Teil der Standard Raspberry Pi OS Installation

**Client**:
- Keine zusätzlichen Abhängigkeiten
- Standard tkinter Widgets

## Geänderte Dateien

1. `Server/GUIServer.py`:
   - ADS7830 Klasse und Initialisierung hinzugefügt
   - `get_battery_voltage()` Funktion hinzugefügt
   - INFO-Protokoll erweitert

2. `Client/GUI.py`:
   - `BATTERY_VOLTAGE` globale Variable hinzugefügt
   - `BATTERY_lab` Label Widget hinzugefügt
   - INFO-Parsing Logik erweitert
   - Farbcodierte Anzeige implementiert

## Konfiguration

Batterie-Konstanten in `GUIServer.py`:
```python
ADCVref = 4.93              # ADC Referenzspannung
battery_channel = 0          # ADC Kanal (0-7)
R15 = 3000                   # Spannungsteiler Widerstand
R17 = 1000                   # Spannungsteiler Widerstand
DivisionRatio = R17 / (R15 + R17)  # = 0.25
```

GUI-Konstanten in `GUI.py`:
```python
Vref = 8.4                   # Volle Batteriespannung
WarningThreshold = 6.0       # Leere Batteriespannung
```

## Fehlerbehebung

### "Battery: N/A" wird angezeigt

**Mögliche Ursachen**:
1. ADS7830 nicht angeschlossen
2. Falsche I²C Adresse (prüfen mit `i2cdetect -y 1`)
3. I²C nicht aktiviert (`sudo raspi-config`)
4. Entwicklungsumgebung ohne Hardware (normal)

### Falsche Spannungswerte

**Prüfen**:
1. Spannungsteiler korrekt? (R15=3kΩ, R17=1kΩ)
2. ADCVref korrekt? (mit Multimeter messen)
3. Batterie tatsächlich mit ADC Eingang verbunden?

### Farbe ändert sich nicht

**Prüfen**:
1. Spannungs-Schwellwerte: 60% = 7.44V, 30% = 6.72V
2. Tkinter Label aktualisiert korrekt?
3. Tatsächliche Spannung mit Multimeter prüfen

## Zukünftige Erweiterungen

- [ ] Popup-Warnung bei niedrigem Batteriestand
- [ ] Batterie-Verlaufs-Diagramm
- [ ] Berechnung der geschätzten Laufzeit
- [ ] Batteriespannungs-Logging in Datei
- [ ] Anpassbare Spannungs-Schwellwerte in GUI

## Siehe auch

- `Examples/07_Voltage/BatteryLevelMonitoring.py` - Standalone Beispiel
- `Server/Voltage.py` - Vollständige Spannungsüberwachung mit Alarmen
- `agents.MD` - Allgemeine System-Dokumentation
