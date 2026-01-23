# FT40 Unit Tests

**Datum:** 2026-01-23  
**Zweck:** Automatisierte Tests für FT40 Implementation (Phase 2-4)

## Übersicht

Diese Unit-Tests validieren die FT40-Implementation und stellen sicher, dass:
- Alle neuen Variablen existieren
- Die Logik korrekt ist
- Keine Regressionen auftreten
- Der Code auf Windows (ohne Hardware) getestet werden kann

## Test-Dateien

### `test_ft40.py` ✅ EMPFOHLEN
**Moderne, fokussierte Tests für FT40**
- 23 Tests in 7 Test-Klassen
- Läuft auf Windows ohne Hardware
- Schnell (< 0.01 Sekunden)
- Fokus auf FT40-spezifische Features

**Ausführen:**
```bash
cd Server
python test_ft40.py
```

### `test_move_baseline.py` ⚠️ LEGACY
**Baseline-Tests (original)**
- Umfassendere Tests
- Erfordert komplexes Mocking
- Kann auf Windows Probleme machen
- Für Referenz beibehalten

## Test-Kategorien

### 1. TestFT40Phase2 - Richtungswechsel-Erkennung
Tests für die Tracking-Variablen:
- `_last_command`
- `_last_speed_sign`
- `_steps_since_change`
- `_direction_changed`

### 2. TestFT40Phase3 - Variable Alpha
Tests für die Alpha-Berechnung:
- Alpha 0.3 → 1.0 über 5 Steps
- Korrekte Mathematik
- Grenzen-Prüfung

### 3. TestFT40Phase4 - Smart Phase-Reset
Tests für Timer-Logik:
- Stop-Counter und Threshold
- Kurzer Stop: Phase bleibt erhalten
- Langer Stop: Phase wird zurückgesetzt

### 4. TestFT40Integration - Integration
Tests für Gesamtfunktionalität:
- `calculate_target_positions()` existiert und funktioniert
- `apply_leg_position()` existiert
- `move_thread()` existiert
- `_leg_positions` Dictionary korrekt

### 5. TestFT40EdgeCases - Edge Cases
Tests für Grenzfälle:
- Phase wrap-around (1.0 → 0.0)
- Alpha-Grenzen (0.0 bis 1.0)
- Counter-Inkrement und Reset

### 6. TestFT40Constants - Konstanten
Tests für definierte Konstanten:
- CYCLE_STEPS
- CMD_FORWARD, CMD_BACKWARD, etc.

### 7. TestFT40Regression - Regression
Tests für alte Funktionalität:
- Alle alten Variablen existieren noch
- Keine Breaking Changes

## Ergebnisse

**Aktueller Stand (2026-01-23):**
```
Tests run: 23
Failures: 0
Errors: 0
Success: True
```

✅ Alle Tests bestanden!

## Hardware-Mocking

Die Tests mocken alle Hardware-Dependencies:
- `mpu6050` - Gyroscope/Accelerometer
- `Adafruit_PCA9685` - PWM Controller
- `RPi.GPIO` - Raspberry Pi GPIO
- `picamera` - Raspberry Pi Camera
- `smbus` - I2C Bus

**Warum?**
- Tests laufen auf Windows ohne Raspberry Pi
- Schnelle Iteration
- CI/CD möglich

## Ausführung

### Einzelner Test-Run
```bash
cd Server
python test_ft40.py
```

### Mit Verbose Output
```python
# In test_ft40.py bereits aktiviert
# Zeigt jeden Test einzeln
```

### Nur bestimmte Test-Klasse
```bash
cd Server
python -m unittest test_ft40.TestFT40Phase4
```

### Nur bestimmter Test
```bash
cd Server
python -m unittest test_ft40.TestFT40Phase4.test_short_stop_logic
```

## Integration in Entwicklungs-Workflow

### Vor jedem Commit
```bash
cd Server
python test_ft40.py
```

Falls alle Tests grün: ✅ Commit OK  
Falls Tests rot: ❌ Fixe zuerst

### Nach jeder Änderung an Move.py
```bash
cd Server
python test_ft40.py
```

Stelle sicher, dass keine Regression aufgetreten ist.

## CI/CD Integration (optional)

Diese Tests können in CI/CD-Pipelines integriert werden:

**GitHub Actions Beispiel:**
```yaml
name: FT40 Tests
on: [push]
jobs:
  test:
    runs-on: windows-latest
    steps:
      - uses: actions/checkout@v2
      - uses: actions/setup-python@v2
        with:
          python-version: '3.7'
      - run: cd Server && python test_ft40.py
```

## Erweitern der Tests

### Neuen Test hinzufügen
```python
class TestFT40Phase5(unittest.TestCase):
    """Tests für Phase 5: Vertikale Interpolation"""
    
    def test_vertical_interpolation(self):
        """Test: Vertikale Position wird interpoliert"""
        # Test-Code hier
        pass
```

### Test-Struktur
1. Test-Klasse mit beschreibendem Namen
2. Docstring mit Beschreibung
3. `setUp()` wenn Zustand vor jedem Test benötigt wird
4. Test-Methoden mit `test_` Präfix
5. Aussagekräftige Assertions

## Bekannte Limitierungen

### Was NICHT getestet wird:
- ❌ Tatsächliche Hardware-Interaktion
- ❌ Servo-Bewegungen
- ❌ PWM-Signale
- ❌ Timing in Echtzeit
- ❌ Threading-Interaktion

### Was getestet wird:
- ✅ Logik und Algorithmen
- ✅ Datenstrukturen
- ✅ Funktions-Existenz
- ✅ Konstanten und Initialisierung
- ✅ Mathematische Berechnungen

### Warum?
Hardware-Tests erfordern echte Hardware und sind langsam. Diese Unit-Tests fokussieren sich auf die **Logik** und können schnell und oft ausgeführt werden.

## Hardware-Tests

Für echte Hardware-Tests:
1. Upload `Move.py` auf Raspberry Pi
2. Führe manuelle Tests durch (siehe `FT40 - Test Checkliste.md`)
3. Beobachte Roboter-Verhalten
4. Prüfe Server-Logs

**Unit-Tests ersetzt NICHT Hardware-Tests!**
Sie ergänzen einander.

## Fehlersuche

### Tests schlagen fehl nach Änderung
1. Prüfe, ob Änderung absichtlich war
2. Falls ja: Update Tests
3. Falls nein: Revert Änderung

### Import-Fehler
```
ModuleNotFoundError: No module named 'mpu6050'
```

**Fix:** Hardware-Module werden automatisch gemockt. Stelle sicher, dass Mocking VOR dem Import passiert.

### Assertion-Fehler
```
AssertionError: Expected 0.3, got 0.29999999999999993
```

**Fix:** Verwende `assertAlmostEqual()` statt `assertEqual()` für Float-Vergleiche.

## Zusammenfassung

✅ **Schnell:** Tests in < 0.01 Sekunden  
✅ **Windows-kompatibel:** Läuft ohne Hardware  
✅ **Fokussiert:** Nur FT40-relevante Tests  
✅ **Wartbar:** Klare Struktur, gute Dokumentation  
✅ **Erweiterbar:** Einfach neue Tests hinzufügen  

**Nutze diese Tests regelmäßig während der Entwicklung!**
