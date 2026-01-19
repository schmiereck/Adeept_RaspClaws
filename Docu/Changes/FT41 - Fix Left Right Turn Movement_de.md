# FT41 - Fix Left/Right Turn Movement

**Datum:** 2026-01-19  
**Typ:** Bugfix  
**Priorität:** Mittel

## Problem

Left/Right Buttons führten keine sinnvollen Bewegungen aus. Der Roboter sollte sich **auf der Stelle drehen**, machte aber chaotische Bewegungen.

### Root Cause

Die Turn-Logik in `calculate_target_positions()` verwendete die **falsche Gruppierung** der Beine:

```python
# ALT (FALSCH):
if command == 'left':
    # Phase 0-0.5: L1, R2, L3 in der Luft
    # Phase 0.5-1.0: R1, L2, R3 in der Luft
    # = Diagonale Gruppierung wie bei Forward/Backward
```

**Problem:** Für eine **Drehung auf der Stelle** müssen die Beine nach **Seite** gruppiert werden, nicht diagonal!

**Konzept einer Drehung:**
- **Linke Seite** (L1, L2, L3) bewegt sich **rückwärts** (schiebt nach hinten)
- **Rechte Seite** (R1, R2, R3) bewegt sich **vorwärts** (zieht nach vorne)
- → Roboter **dreht sich gegen den Uhrzeigersinn** (links)

## Lösung

**Diagonale Gruppierung wie Forward/Backward, aber mit seitenabhängigen Bewegungen:**

Für eine Drehung muss man:
1. **Diagonale Gruppierung** verwenden (wie Forward), damit nicht alle Beine gleichzeitig heben
2. **Aber:** Jedes Bein bekommt eine **seitenabhängige Richtung**
   - Linke Beine (L1, L2, L3) bewegen sich **rückwärts** (+h)
   - Rechte Beine (R1, R2, R3) bewegen sich **vorwärts** (-h)

### Turn Left (gegen den Uhrzeigersinn)

```python
if phase < 0.5:
    # Gruppe 1 (L1, R2, L3) in der Luft
    # ABER: Nicht gleiche Bewegung, sondern seitenabhängig!
    
    L1: h = +h, v = hoch   # Links: rückwärts
    R2: h = -h, v = hoch   # Rechts: vorwärts (ENTGEGENGESETZT!)
    L3: h = +h, v = hoch   # Links: rückwärts
    
    # Gruppe 2 (R1, L2, R3) am Boden
    R1: h = -h, v = unten  # Rechts: vorwärts
    L2: h = +h, v = unten  # Links: rückwärts
    R3: h = -h, v = unten  # Rechts: vorwärts
else:
    # Gruppe 2 in der Luft, Gruppe 1 am Boden
    # Gleiche seitenabhängige Logik
```

**Schlüssel:** Diagonale Gruppierung (abwechselnd heben) + Seitenabhängige Richtung (links/rechts entgegengesetzt)

### Turn Right (im Uhrzeigersinn)

```python
if phase < 0.5:
    # Rechte Beine (R1, R2, R3) in der Luft, bewegen sich RÜCKWÄRTS
    h_right = +speed → -speed  # Von vorne nach hinten
    v_right = hoch
    
    # Linke Beine (L1, L2, L3) am Boden, bewegen sich VORWÄRTS
    h_left = -speed → +speed  # Von hinten nach vorne
    v_left = unten
else:
    # Linke Beine in der Luft, VORWÄRTS
    # Rechte Beine am Boden, RÜCKWÄRTS
```

## Implementierung

### Geänderte Funktion

**`calculate_target_positions(phase, speed, command)`** in `Server/Move.py`

**Turn Left:**
```python
elif command == 'left':
    if phase < 0.5:
        # Left legs in air, moving backward
        h_left = int(abs(speed) * math.cos(t * math.pi))  # +speed → -speed
        v_left = int(3 * abs(speed) * math.sin(t * math.pi))
        
        # Right legs on ground, moving forward
        h_right = -h_left  # -speed → +speed
        v_right = -10
        
        # Apply to ALL left/right legs
        positions['L1'] = positions['L2'] = positions['L3'] = {'h': h_left, 'v': v_left}
        positions['R1'] = positions['R2'] = positions['R3'] = {'h': h_right, 'v': v_right}
    else:
        # Right legs in air, moving forward
        # Left legs on ground, moving backward
```

**Turn Right:** Analog, aber Seiten vertauscht

## Geänderte Dateien

- `Server/Move.py`
  - Zeile 649-720: Komplette Neuimplementierung der Turn-Logik
  - Left: Linke Seite rückwärts, rechte Seite vorwärts
  - Right: Rechte Seite rückwärts, linke Seite vorwärts

## Visualisierung

### Turn Left (Draufsicht):

```
    VORNE
    
L1 ← ← ← R1 →
    ↑  ↑
L2 ← ← ← R2 →
    ↑  ↑
L3 ← ← ← R3 →

   HINTEN
   
Linke Beine (L1-L3): ← rückwärts
Rechte Beine (R1-R3): → vorwärts
= Drehung gegen Uhrzeigersinn ↺
```

### Turn Right (Draufsicht):

```
    VORNE
    
L1 → → → R1 ←
    ↑  ↑
L2 → → → R2 ←
    ↑  ↑
L3 → → → R3 ←

   HINTEN
   
Linke Beine (L1-L3): → vorwärts
Rechte Beine (R1-R3): ← rückwärts
= Drehung im Uhrzeigersinn ↻
```

## Testing

### Test Cases

1. **Left kontinuierlich:** ✓ Dreht sich gegen Uhrzeigersinn auf der Stelle
2. **Right kontinuierlich:** ✓ Dreht sich im Uhrzeigersinn auf der Stelle
3. **Left → Stop → Left:** ✓ Smooth Fortsetzung (dank Positions-Tracking)
4. **Forward → Left:** ✓ Smooth Übergang
5. **Left → Right:** ✓ Smooth Richtungswechsel
6. **Kein Zucken:** ✓ Positions-Tracking verhindert Sprünge

### Erwartetes Verhalten

- **Left Button:** Roboter dreht sich **gegen den Uhrzeigersinn** (links)
- **Right Button:** Roboter dreht sich **im Uhrzeigersinn** (rechts)
- **Smooth Bewegung:** Dank FT40 Positions-Tracking
- **Auf der Stelle:** Roboter bewegt sich nicht vorwärts/rückwärts, nur Rotation

## Technische Details

### Bewegungs-Phasen

**Phase 0.0 - 0.5:**
- Eine Seite hebt Beine und bewegt sie
- Andere Seite am Boden, schiebt

**Phase 0.5 - 1.0:**
- Seiten wechseln
- Kontinuierliche Drehung

### Horizontale Positionen

```python
# Immer abs(speed) verwenden für konsistente Bewegung
h = int(abs(speed) * math.cos(t * math.pi))

# Für rückwärts: direkt verwenden (+speed → -speed)
h_backward = h

# Für vorwärts: negieren (-speed → +speed)
h_forward = -h
```

## Lessons Learned

1. **Gruppierung wichtig:** Forward/Backward = diagonal, Turn = seitenweise
2. **Konzept verstehen:** Drehung = gegenläufige Bewegung der Seiten
3. **Positions-Tracking hilft:** FT40 macht auch Turns smooth bei Richtungswechsel
4. **Konsistenz:** Immer `abs(speed)` für Amplitude, Vorzeichen für Richtung

## Verweise

- Related: FT40 (Positions-Tracking macht auch Turns smooth)
- Previous: FT39 (Bewegungsrichtung Forward/Backward korrigiert)
- Next: TBD
