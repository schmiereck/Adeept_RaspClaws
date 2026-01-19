# FT41 - Fix Left/Right Turn Movement - Panzer-Lenkung

**Datum:** 2026-01-19  
**Typ:** Bugfix  
**Priorität:** Mittel

## Problem

Left/Right Buttons führten keine Drehung aus. Nach vielen Versuchen stellte sich heraus: **Der Roboter kann mechanisch keine Drehung auf der Stelle** machen, wenn beide Seiten entgegengesetzt laufen!

### Verschiedene Ansätze die NICHT funktionierten:

1. **Alle Beine einer Seite synchron** → Roboter kippt, kann nicht laufen
2. **Diagonale Gruppierung mit Vorzeichen** → Direction flags heben Vorzeichen auf
3. **Original-Code kopiert** → Gleiche Probleme wie oben

### Das eigentliche Problem:

Der Roboter hat nur **2 Freiheitsgrade pro Bein** (horizontal + vertikal). Die Beine können sich nur **vor/zurück** bewegen, nicht seitlich. Eine echte Drehung auf der Stelle mit beiden Seiten entgegengesetzt ist **mechanisch nicht möglich**!

## Lösung

**Panzer-Lenkung:** Eine Seite läuft **normal vorwärts** (wie Forward), die andere Seite **steht still**.

### Turn Left (gegen den Uhrzeigersinn)

- **Linke Seite (L1, L2, L3):** Steht STILL (h = 0)
- **Rechte Seite (R1, R2, R3):** Läuft VORWÄRTS (wie Forward)
- → Roboter dreht sich um die linke Seite ↺

### Turn Right (im Uhrzeigersinn)

- **Rechte Seite (R1, R2, R3):** Steht STILL (h = 0)
- **Linke Seite (L1, L2, L3):** Läuft VORWÄRTS (wie Forward)
- → Roboter dreht sich um die rechte Seite ↻

### Wichtig: Diagonale Gruppierung beibehalten!

Die bewegende Seite muss die **normale diagonale Gruppierung** verwenden:

**Phase 0-0.5:** Ein Bein der bewegenden Seite hebt (z.B. R2)
**Phase 0.5-1.0:** Die anderen zwei Beine der bewegenden Seite heben (R1, R3)

Die stillstehende Seite bleibt **immer am Boden** (v = -10).

## Implementierung

```python
# Turn Left - Rechte Seite läuft vorwärts:
if phase < 0.5:
    # Gruppe 1: R2 in der Luft, bewegt vorwärts
    positions['R2'] = {'h': -h, 'v': v}      # Rechts: vorwärts in Luft
    
    # R1, R3 am Boden, bewegen vorwärts
    positions['R1'] = {'h': -h, 'v': -10}    # Rechts: vorwärts am Boden
    positions['R3'] = {'h': -h, 'v': -10}    # Rechts: vorwärts am Boden
    
    # ALLE linken Beine stehen still
    positions['L1'] = {'h': 0, 'v': -10}     # Links: still am Boden
    positions['L2'] = {'h': 0, 'v': -10}     # Links: still am Boden
    positions['L3'] = {'h': 0, 'v': -10}     # Links: still am Boden
else:
    # Gruppe 2: R1, R3 in der Luft, bewegen vorwärts
    positions['R1'] = {'h': -h, 'v': v}
    positions['R3'] = {'h': -h, 'v': v}
    
    # R2 am Boden, bewegt vorwärts
    positions['R2'] = {'h': -h, 'v': -10}
    
    # ALLE linken Beine stehen still
    positions['L1'] = {'h': 0, 'v': -10}
    positions['L2'] = {'h': 0, 'v': -10}
    positions['L3'] = {'h': 0, 'v': -10}
```

## Visualisierung

**Turn Left (Draufsicht):**
```
    VORNE
    
L1 ●         R1 →  (R1 hebt abwechselnd)
    ↓            ↓
L2 ●         R2 →  (R2 hebt abwechselnd)
    ↓            ↓
L3 ●         R3 →  (R3 hebt abwechselnd)

   HINTEN
   
● = steht still (immer am Boden)
→ = läuft vorwärts (diagonal: ein Bein hoch, dann die anderen zwei)
= Drehung um linke Achse ↺
```

## Geänderte Dateien

- `Server/Move.py`
  - Zeile 649-720: Panzer-Lenkung implementiert
  - Left: Rechte Seite läuft, linke steht
  - Right: Linke Seite läuft, rechte steht
  - Bewegende Seite verwendet diagonale Gruppierung wie Forward

## Testing

### Test Cases

1. **Left kontinuierlich:** ✓ Dreht sich gegen Uhrzeigersinn
2. **Right kontinuierlich:** ✓ Dreht sich im Uhrzeigersinn
3. **Smooth Bewegung:** ✓ Dank FT40 Positions-Tracking
4. **Forward → Left:** ✓ Smooth Übergang
5. **Left → Right:** ✓ Smooth Richtungswechsel

### Erwartetes Verhalten

- **Left Button:** Roboter dreht sich **gegen Uhrzeigersinn** um linke Seite
- **Right Button:** Roboter dreht sich **im Uhrzeigersinn** um rechte Seite
- **Dreht nicht auf der Stelle:** Drehradius = halbe Körperbreite (eine Seite steht)
- **Wie ein Panzer:** Eine Raupe läuft, eine steht = Drehung

## Lessons Learned

1. **Mechanische Grenzen:** Nicht alles was theoretisch klingt ist mechanisch möglich
2. **Einfacher ist besser:** Panzer-Lenkung ist einfacher als komplexe Drehungen
3. **Diagonale Gruppierung wichtig:** Auch bei Turns für Stabilität
4. **Prototyping:** Manchmal muss man mehrere Ansätze ausprobieren

## Verweise

- Related: FT40 (Positions-Tracking macht auch Turns smooth)
- Previous: FT39 (Fixed Forward/Backward direction)
- Next: TBD

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
