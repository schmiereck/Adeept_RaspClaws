# FT40 - Smooth Richtungswechsel mit Positions-Tracking

**Datum:** 2026-01-19  
**Typ:** Feature Enhancement  
**Priorität:** Hoch

## Problem

Beim Wechsel zwischen Forward und Backward gab es ein **Zucken**, obwohl die Bewegung innerhalb einer Richtung smooth war.

### Root Cause

Die bisherige Lösung verwendete eine **globale Phase** (`_movement_phase`), die beim Richtungswechsel zurückgesetzt wurde:

```python
# ALT (FT38-39):
_movement_phase = 0.0  # Globale Phase

# Bei Richtungswechsel:
if direction_changed:
    _movement_phase = 0.0  # Reset → Beine springen zur Start-Position!
```

**Problem:** Die Phase wurde zurückgesetzt, aber die **tatsächlichen Bein-Positionen** wurden nicht berücksichtigt. Die Beine sprangen zur Start-Position des neuen Zyklus, was zu einem sichtbaren Zucken führte.

**Beispiel:**
- Forward bei phase=0.7: Bein bei Position +20 (fast vorne)
- Backward starten: Phase auf 0.0 → Bein soll bei -35 sein
- **Sprung von +20 zu -35!**

## Lösung

**Positions-basiertes Tracking statt Phase-Tracking:**

Speichere die **tatsächlichen Servo-Positionen** jedes Beins und interpoliere smooth von aktueller Position zur Ziel-Position bei Richtungswechsel.

### Neue Datenstruktur

```python
# Globales Dictionary für tatsächliche Bein-Positionen
_leg_positions = {
    'L1': 0,  # Left front horizontal position
    'L2': 0,  # Left middle horizontal position
    'L3': 0,  # Left rear horizontal position
    'R1': 0,  # Right front horizontal position
    'R2': 0,  # Right middle horizontal position
    'R3': 0   # Right rear horizontal position
}
```

### Algorithmus

```python
def move_smooth(speed, command, cycle_steps=30):
    # 1. Berechne Ziel-Positionen für aktuelle Phase
    target_positions = calculate_target_positions(phase, speed, command)
    
    # 2. Interpoliere von aktueller zu Ziel-Position
    for leg in ['L1', 'L2', 'L3', 'R1', 'R2', 'R3']:
        current = _leg_positions[leg]
        target = target_positions[leg]['h']
        
        # Smooth interpolation
        if command_changed and step < 5:
            alpha = 0.3 + (step / 5) * 0.7  # Stärkere Interpolation am Anfang
        else:
            alpha = 1.0  # Normal: direkt zur Ziel-Position
        
        new_position = current + alpha * (target - current)
        _leg_positions[leg] = new_position
        
        # 3. Wende Position auf Servo an
        apply_leg_position(leg, new_position, vertical)
```

### Vorteile

1. **Smooth Richtungswechsel:** Keine Sprünge mehr, Beine interpolieren von aktueller Position
2. **Smooth Stop/Start:** Beine bleiben an ihrer Position, kein Reset
3. **Flexible Interpolation:** Stärkere Interpolation in den ersten Steps nach Änderung
4. **Unabhängig von Phase:** Position ist die "Source of Truth", nicht die Phase

## Implementierung

### Neue Funktionen

**`calculate_target_positions(phase, speed, command)`**
- Berechnet Ziel-Positionen für alle 6 Beine bei gegebener Phase
- Unterstützt forward/backward ('no'), left, right
- Gibt Dictionary zurück: `{'L1': {'h': ..., 'v': ...}, ...}`

**`apply_leg_position(leg, horizontal, vertical)`**
- Wendet Position auf ein spezifisches Bein an
- Ruft entsprechende `dove_Left_X()` / `dove_Right_X()` Funktion auf

### Geänderte Funktionen

**`move_smooth(speed, command, cycle_steps=30)`**
- Verwendet jetzt `_leg_positions` Dictionary statt `_movement_phase`
- Erkennt Kommando- und Richtungswechsel
- Interpoliert smooth von aktuell → Ziel
- Stärkere Interpolation (alpha = 0.3 → 1.0) in den ersten 5 Steps nach Wechsel

### Neue Imports

```python
import math  # Für cos/sin Berechnungen in calculate_target_positions()
```

## Geänderte Dateien

- `Server/Move.py`
  - Zeile 7: `import math` hinzugefügt
  - Zeile 518-530: Neue globale Variablen (`_leg_positions`, `_last_command`, `_last_speed_sign`)
  - Zeile 532-583: Neue `move_smooth()` mit Positions-Tracking
  - Zeile 586-701: Neue Hilfsfunktionen `calculate_target_positions()` und `apply_leg_position()`

## Testing

### Test Cases

1. **Forward kontinuierlich:** ✓ Smooth ohne Sprünge
2. **Backward kontinuierlich:** ✓ Smooth ohne Sprünge
3. **Forward → Stop → Forward:** ✓ Smooth Fortsetzung
4. **Forward → Backward:** ✓ **Smooth Interpolation, kein Zucken!**
5. **Backward → Forward:** ✓ **Smooth Interpolation, kein Zucken!**
6. **Forward → Left:** ✓ Smooth Übergang
7. **Multiple Richtungswechsel:** ✓ Immer smooth

### Erwartetes Verhalten

- **Bei Richtungswechsel:** Beine interpolieren smooth von ihrer aktuellen Position zur neuen Ziel-Position
- **Erste 5 Steps:** Stärkere Interpolation (alpha 0.3 → 1.0) für smooth Übergang
- **Danach:** Normale Bewegung mit direkter Ziel-Position (alpha = 1.0)
- **Kein Zucken** oder Sprünge mehr bei Richtungswechsel!

## Technische Details

### Interpolations-Formel

```python
new_position = current + alpha * (target - current)
```

- `alpha = 0.3`: 30% Bewegung Richtung Ziel (sehr smooth)
- `alpha = 1.0`: 100% Bewegung Richtung Ziel (direkt)
- `alpha = 0.3 → 1.0`: Gradueller Übergang über 5 Steps

### Positions-Berechnung

Forward/Backward (command='no'):
```python
# Phase 0-0.5: Gruppe 1 (L1, R2, L3) in der Luft
h1 = int(speed * cos(phase * 2 * π))  # Mit Vorzeichen für Richtung
v1 = int(3 * abs(speed) * sin(phase * 2 * π))  # Immer positiv

# Gruppe 2 (R1, L2, R3) am Boden
h2 = -h1  # Entgegengesetzt
v2 = -10  # Am Boden
```

## Lessons Learned

1. **Position > Phase:** Speichere tatsächliche Zustände, nicht abstrakte Phase-Werte
2. **Smooth Transitions:** Interpolation ist besser als Reset
3. **Source of Truth:** Die Servo-Positionen sind der tatsächliche Zustand
4. **Gradual Changes:** Stärkere Interpolation am Anfang hilft bei smooth Übergängen

## Nächste Schritte

- [ ] Optional: Speed-Parameter für variable Geschwindigkeiten
- [ ] Optional: Adaptive Interpolation basierend auf Positions-Differenz
- [ ] Optional: Positions-Tracking auch für vertikale Achse (derzeit nur horizontal)

## Verweise

- Vorheriges Feature: FT39 (Bewegungsrichtung mit abs() für vertikale Bewegung)
- Nächstes Feature: TBD
- Related: FT38 (Verhindere steady() während Bewegung)
