# FT40 Phase 2 - Richtungswechsel-Erkennung

**Datum:** 2026-01-23  
**Status:** ✅ Implementiert  
**Risiko:** Niedrig

## Implementierung

### Neue Variablen

Hinzugefügt in Zeile ~118:
```python
_steps_since_change = 0  # Counter for steps since last direction/command change
_direction_changed = False  # Flag indicating if direction recently changed
```

### Erkennungslogik

Hinzugefügt in `move_thread()` nach der Geschwindigkeitsberechnung (~Zeile 1037-1067):

```python
# Detect command change (forward/backward/left/right)
current_command = (direction_command, turn_command)
command_changed = (current_command != _last_command) and (_last_command is not None)

# Detect direction change (forward ↔ backward flip based on speed sign)
current_direction_value = (speed_left + speed_right) / 2.0
current_sign = 1 if current_direction_value < 0 else (-1 if current_direction_value > 0 else 0)

direction_changed = (current_sign != 0 and 
                    _last_speed_sign != 0 and 
                    current_sign != _last_speed_sign)

# Update tracking variables
if command_changed or direction_changed:
    _steps_since_change = 0
    _direction_changed = True
    # LOG messages for debugging
else:
    _steps_since_change += 1
    if _steps_since_change > 10:
        _direction_changed = False

_last_command = current_command
_last_speed_sign = current_sign
```

## Funktionsweise

1. **Command Change Detection:**
   - Vergleicht aktuelles `(direction_command, turn_command)` mit vorherigem
   - Erkennt z.B. Forward → Left, Backward → Right

2. **Direction Change Detection:**
   - Berechnet Vorzeichen der durchschnittlichen Geschwindigkeit
   - Erkennt Forward (negativ) ↔ Backward (positiv) Wechsel

3. **Counter Management:**
   - Bei Änderung: Reset auf 0, Flag auf True
   - Nach 10 Steps ohne Änderung: Flag auf False

## Verhaltensänderung

**KEINE!** Diese Phase ändert nur die Erkennung, nicht das Bewegungsverhalten.

Die Interpolation verwendet weiterhin konstant `alpha = 0.9`.

## Logging

Bei Richtungswechsel erscheinen Log-Meldungen:
```
[FT40] Direction change detected: -1 → 1
[FT40] Command change detected: (1, 0) → (0, 2)
```

## Tests

### Manuelle Tests durchgeführt:
- [ ] Forward kontinuierlich: Keine Logs, Verhalten unverändert
- [ ] Forward → Backward: Log erscheint
- [ ] Backward → Forward: Log erscheint
- [ ] Forward → Left: Log erscheint
- [ ] Keine Regression bei allen Baseline-Tests

### Erwartetes Verhalten:
- ✅ Logs erscheinen bei Richtungswechsel
- ✅ Keine Verhaltensänderung der Bewegung
- ✅ Keine Regression

## Geänderte Dateien

- `Server/Move.py`
  - Zeile ~118-119: Neue Variablen
  - Zeile ~965: Global declaration erweitert
  - Zeile ~1037-1067: Erkennungslogik hinzugefügt

## Nächste Schritte

Nach erfolgreichen Tests:
- **Phase 3:** Variable Alpha basierend auf `_direction_changed` und `_steps_since_change`

## Rollback

Falls Probleme auftreten:
```powershell
Copy-Item "Move.py.backup_before_ft40_phase2" -Destination "Move.py"
```
