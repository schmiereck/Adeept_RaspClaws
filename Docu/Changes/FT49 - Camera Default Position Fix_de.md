# FT49 - Kamera Standard-Position Fix

**Feature**: Kamera Home-Position respektiert CAMERA_UP_DOWN_DEFAULT Änderungen  
**Datum**: 2026-02-22  
**Status**: ✅ Implementiert

## Problem

Wenn `CAMERA_UP_DOWN_DEFAULT` in Move.py geändert wurde (z.B. von 240 auf 200),
wurde die Kamera Home-Position nach Server-Neustart nicht aktualisiert.

**Ursache**:
- `pwm12` und `pwm13` Variablen werden **einmalig** beim Modul-Import gesetzt:
  ```python
  pwm12 = CAMERA_LEFT_RIGHT_DEFAULT  # Beim Import gesetzt
  pwm13 = CAMERA_UP_DOWN_DEFAULT     # Beim Import gesetzt
  ```
- `init_all()` und `look_home()` verwendeten diese **statischen** Variablen
- Bei Änderung der Konstanten blieben die alten Werte bestehen
- Server nutzt Stromspar-Modus: Servos bleiben beim Start "weich"
- Kamera-Servos werden erst aktiviert, wenn GUI verbindet oder Befehle sendet
- Zu diesem Zeitpunkt war der alte `pwm13` Wert bereits "eingebacken"

## Lösung

`init_all()` und `look_home()` geändert, um die **Konstanten direkt** zu verwenden
statt der statischen `pwm12`/`pwm13` Variablen.

### Änderungen in init_all()

**Vorher**:
```python
pwm.channels[12].duty_cycle = _pulse_to_duty_cycle(pwm12)
pwm.channels[13].duty_cycle = _pulse_to_duty_cycle(pwm13)
```

**Nachher**:
```python
# Konstanten direkt für Kamera-Servos verwenden
pwm.channels[12].duty_cycle = _pulse_to_duty_cycle(CAMERA_LEFT_RIGHT_DEFAULT)
pwm.channels[13].duty_cycle = _pulse_to_duty_cycle(CAMERA_UP_DOWN_DEFAULT)

# Tracking-Variablen aktualisieren
Left_Right_input = CAMERA_LEFT_RIGHT_DEFAULT
Up_Down_input = CAMERA_UP_DOWN_DEFAULT
```

### Änderungen in look_home()

**Vorher**:
```python
default_positions = [
    pwm0, pwm1, ..., pwm11,
    pwm12, pwm13, pwm14, pwm15  # Statische Werte vom Import
]
```

**Nachher**:
```python
default_positions = [
    pwm0, pwm1, ..., pwm11,
    CAMERA_LEFT_RIGHT_DEFAULT,  # Immer aktueller Wert
    CAMERA_UP_DOWN_DEFAULT,     # Immer aktueller Wert
    pwm14, pwm15
]
```

## Vorteile

1. ✅ **Dynamisch**: Verwendet immer aktuelle Konstantenwerte
2. ✅ **Vorhersagbar**: Änderungen an Konstanten wirken sofort
3. ✅ **Wartbar**: Keine Verfolgung statischer Variablen nötig
4. ✅ **Konsistent**: Gleiches Verhalten in `init_all()` und `look_home()`

## Test

1. `CAMERA_UP_DOWN_DEFAULT` in Move.py ändern
2. Änderungen committen und pushen
3. Auf Raspberry Pi:
   ```bash
   git pull
   sudo systemctl restart gui_server.service
   ```
4. GUI verbinden und "Home"-Button drücken
5. Kamera sollte sich zur NEUEN Standard-Position bewegen ✓

## Geänderte Dateien

- **Server/Move.py**:
  - `init_all()`: Konstanten direkt für Servos 12/13 verwenden
  - `look_home()`: Konstanten direkt für Servos 12/13 verwenden

## Hinweise

- Bein-Servos (pwm0-pwm11) verwenden weiterhin statische Variablen (beabsichtigt)
- Nur Kamera-Servos benötigen dynamisches Verhalten für einfache Anpassung
- Stromspar-Modus funktioniert weiterhin korrekt
- Rückwärtskompatibel mit bestehendem Code

## Warum nur Kamera-Servos?

Kamera-Servos werden häufig für verschiedene Robot-Konfigurationen angepasst:
- Unterschiedliche Kamera-Montagewinkel
- Unterschiedliche Servo-Marken/Toleranzen
- Benutzer-Präferenzen für Standard-Blickwinkel

Bein-Servos benötigen selten Anpassung und profitieren von Compile-Zeit-Konstanten.
