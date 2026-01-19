# Bugfix: Zucken im Smooth-Modus behoben

**Datum:** 2026-01-19  
**Problem:** Im Smooth-Modus (slow) zucken die Beine während der Luftphase zurück zur Mitte  
**Ursache:** Fehlerhafte Interpolation in der `dove()` Funktion

---

## Problem-Beschreibung

### Symptome
- **Normal-Modus:** Bewegungen sehen gut aus ✓
- **Smooth-Modus (slow):** Beine zucken während der Luftphase kurz in die Gegenrichtung (zur Mitte)

### Beobachtung
Wenn ein Bein nach vorne oder hinten bewegt wird (in der Luft), zuckt es mitten im langsamen Bewegungsablauf einmal kurz zur Mitte und macht dann mit der weichen Bewegung weiter.

---

## Ursachen-Analyse

### Code-Pfade
Der Roboter verwendet **zwei verschiedene Bewegungsfunktionen**:
- **Normal-Modus:** `move()` → verwendet `left_I()`, `right_I()` usw. → verwendet `leg_control()`
- **Smooth-Modus:** `execute_movement_step()` mit `SmoothMode=True` → verwendet `dove()` → verwendet `dove_Left_I()`, `dove_Right_I()` usw.

### Das Problem in der `dove()` Funktion (Move.py, Zeile 490-720)

**Falsche Interpolation (vorher):**
```python
for speed_I in range(0, (speed+int(speed/dpi)), int(speed/dpi)):
    speed_II = speed_I          # Speichere Original
    speed_I = speed - speed_I   # ← UMKEHRUNG! Überschreibt speed_I!
    
    dove_Left_I(-speed_I, 3*speed_II)  # Verwendet umgekehrten Wert
```

**Was passierte mit speed=35, dpi=10:**
| Loop | Original speed_I | Nach Umkehrung | Position horizontal |
|------|-----------------|----------------|---------------------|
| 1    | 0               | 35             | -35 (maximal links) |
| 2    | 10              | 25             | -25 (zur Mitte!)    |
| 3    | 20              | 15             | -15 (noch mehr Mitte!) |
| 4    | 30              | 5              | -5 (fast Mitte)     |

Das Bein bewegte sich: **-35 → -25 → -15 → -5** (zur Mitte zurück!) ← **ZUCKEN!** ❌

---

## Lösung

### Korrekte lineare Interpolation (jetzt)

**Step Input 1 (Zeile 501-538):**
```python
for speed_I in range(0, (speed+int(speed/dpi)), int(speed/dpi)):
    # Linear interpolation: speed_I goes from 0 to speed
    # Vertical position: gradually increase (lift leg)
    dove_Left_I(-speed_I, 3*speed_I)  # Direkt verwenden, keine Umkehrung!
```

**Was passiert jetzt mit speed=35, dpi=10:**
| Loop | speed_I | Position horizontal | Bewegung |
|------|---------|---------------------|----------|
| 1    | 0       | 0                   | Neutral  |
| 2    | 10      | -10                 | Nach links ✓ |
| 3    | 20      | -20                 | Weiter links ✓ |
| 4    | 30      | -30                 | Noch weiter ✓ |

Das Bein bewegt sich jetzt: **0 → -10 → -20 → -30** (linear nach links!) ✓

### Alle Schritte korrigiert

**Step Input 1:** Linear von 0 zu speed  
**Step Input 2:** Linear von speed zu 0 (absteigend)  
**Step Input 3:** Linear von 0 zu speed  
**Step Input 4:** Linear von speed zu 0 (absteigend)  

**Rückwärtsbewegung (speed < 0):** Alle 4 Schritte ebenfalls korrigiert

---

## Geänderte Dateien

### `Server/Move.py`

**Zeile 501-538** (Step Input 1 - Forward):
- Entfernt: `speed_II = speed_I; speed_I = speed - speed_I`
- Neu: Direkte Verwendung von `speed_I` ohne Umkehrung

**Zeile 539-573** (Step Input 2 - Forward):
- Verwendet jetzt: `speed - speed_I` für absteigende Interpolation

**Zeile 574-608** (Step Input 3 - Forward):
- Direkte lineare Interpolation

**Zeile 609-643** (Step Input 4 - Forward):
- Absteigende Interpolation mit `speed - speed_I`

**Zeile 653-711** (Step Input 1-4 - Backward):
- Alle 4 Schritte korrigiert für Rückwärtsbewegung

---

## Test-Ergebnisse

### Vor dem Fix
- ✅ Normal-Modus: OK
- ❌ Smooth-Modus: **Zucken** während der Bewegung (Bein zieht zur Mitte zurück)

### Nach dem Fix
- ✅ Normal-Modus: OK
- ✅ Smooth-Modus: **Keine Zuckbewegung mehr**, fließende lineare Bewegungen

---

## Technische Details

### Warum die Umkehrung falsch war

Die ursprüngliche Logik versuchte:
```python
speed_II = speed_I         # Vertikale Position (Höhe)
speed_I = speed - speed_I  # Horizontale Position (umgekehrt)
```

**Problem:** `speed - speed_I` ergibt eine **absteigende** Reihe:
- 35, 25, 15, 5 (wenn speed=35, dpi=10)

Aber das Bein sollte **aufsteigend** bewegt werden:
- 0, 10, 20, 30

### Die korrekte lineare Interpolation

**Aufsteigend (0 → speed):**
```python
for speed_I in range(0, speed, int(speed/dpi)):
    position = speed_I  # 0, 10, 20, 30...
```

**Absteigend (speed → 0):**
```python
for speed_I in range(0, speed, int(speed/dpi)):
    position = speed - speed_I  # 35, 25, 15, 5...
```

Aber **nur wenn das gewünscht ist**! Nicht versehentlich!

---

## Verwandte Änderungen

Siehe auch:
- `ENHANCEMENT_SMOOTH_SERVOS_2026-01-19.md` - Smooth-Modus Implementierung
- `REFACTORING_LEG_FUNCTIONS_2026-01-19.md` - Leg control refactoring

---

## Status

✅ **Behoben** - Smooth-Modus funktioniert jetzt ohne Zuckbewegungen durch korrekte lineare Interpolation

---

## Problem-Beschreibung

### Symptome
- **Normal-Modus:** Bewegungen sehen gut aus ✓
- **Smooth-Modus (slow):** Beine zucken während der Luftphase kurz in die Gegenrichtung (zur Mitte)

### Beobachtung
Wenn ein Bein nach vorne oder hinten bewegt wird (in der Luft), zuckt es mitten im langsamen Bewegungsablauf einmal kurz zur Mitte und macht dann mit der weichen Bewegung weiter.

---

## Ursachen-Analyse

### Bewegungszyklus
```
            pos=1 (oben, Mitte - Bein in der Luft)
           /     \
          /       \
         /         \
    pos=2-----------pos=4  (am Boden)
   (vorne)         (hinten)
         \         /
          \       /
           \     /
            pos=3 (am Boden)
```

**Schrittfolge:** 1 → 2 → 3 → 4 → (zurück zu 1)

### Das Problem in `leg_control()` (Move.py, Zeile 273, 303)

**Falsche Implementierung (vorher):**
```python
elif pos == 3:
    # Position 3: Zur MITTE zurück! <- FALSCH!
    pwm.set_pwm(h_channel, 0, base_h + int(wiggle/2))  # Zurück zur Mitte
```

**Was passierte:**
1. **pos=1**: Bein oben, Mitte (in der Luft)
2. **pos=2**: Bein vorne, unten (am Boden) ✓
3. **pos=3**: Bein **zurück zur Mitte** ← **ZUCKEN!** ❌
4. **pos=4**: Bein hinten, unten (am Boden) ✓

Das Bein bewegte sich also:
- Von **vorne (pos=2)** 
- Zurück zur **Mitte (pos=3)** ← sichtbares Zucken!
- Dann erst nach **hinten (pos=4)**

---

## Lösung

### Korrekte Implementierung (jetzt)

**Forward Direction (Zeile 273):**
```python
elif pos == 3:
    # Position 3: Neutral position (smooth transition from pos 2 to pos 4)
    pwm.set_pwm(h_channel, 0, base_h)  # Neutral position
    if height_flag:
        pwm.set_pwm(v_channel, 0, base_v - height_change)
    else:
        pwm.set_pwm(v_channel, 0, base_v + height_change)
```

**Reverse Direction (Zeile 303):**
```python
elif pos == 3:
    # Position 3: Neutral position (smooth transition)
    pwm.set_pwm(h_channel, 0, base_h)  # Neutral position
    if height_flag:
        pwm.set_pwm(v_channel, 0, base_v - wiggle)
    else:
        pwm.set_pwm(v_channel, 0, base_v + wiggle)
```

### Neue Bewegungssequenz

**pos=2 → pos=3 → pos=4:**
- **pos=2**: `base_h + wiggle` (maximal vorne)
- **pos=3**: `base_h` (neutral/Mitte) ← **fließender Übergang**
- **pos=4**: `base_h - wiggle` (maximal hinten)

**Resultat:**
- Gleichmäßige Bewegung von vorne nach hinten
- **Kein Zurückzucken** mehr!
- Smooth-Modus funktioniert jetzt korrekt

---

## Geänderte Dateien

### `Server/Move.py`

**Zeile 273-284** (Forward Direction):
```python
elif pos == 3:
    # Position 3: Continue backward motion (neutral position)
    pwm.set_pwm(h_channel, 0, base_h)  # Neutral position
    # ... height logic unchanged
```

**Zeile 303-314** (Reverse Direction):
```python
elif pos == 3:
    # Position 3: Continue forward motion (neutral position)
    pwm.set_pwm(h_channel, 0, base_h)  # Neutral position
    # ... height logic unchanged
```

---

## Test-Ergebnisse

### Vor dem Fix
- ❌ Normal-Modus: OK
- ❌ Smooth-Modus: **Zucken** während der Bewegung

### Nach dem Fix
- ✅ Normal-Modus: OK
- ✅ Smooth-Modus: **Keine Zuckbewegung mehr**, fließende Bewegungen

---

## Technische Details

### Bewegungszyklus erklärt

**Position 1** (Luftphase):
- Bein ist **oben** (height_change * 3)
- Horizontal in Mittelposition (`base_h`)
- Bein wird angehoben und nach vorne/hinten bewegt

**Position 2** (Bodenphase - maximal vorne/hinten):
- Bein ist **unten** (am Boden)
- Horizontal maximal ausgelenkt (`base_h ± wiggle`)

**Position 3** (Bodenphase - Übergang):
- Bein ist **unten** (am Boden)
- Horizontal in **Neutralposition** (`base_h`)
- Sanfter Übergang von pos 2 zu pos 4

**Position 4** (Bodenphase - maximal hinten/vorne):
- Bein ist **unten** (am Boden)
- Horizontal maximal ausgelenkt in Gegenrichtung (`base_h ∓ wiggle`)

### Warum das Zucken auftrat

Das alte `base_h + int(wiggle/2)` bzw. `base_h - int(wiggle/2)` führte zu:
- pos=2: `base_h + wiggle` (z.B. 300 + 35 = 335)
- pos=3: `base_h + wiggle/2` (z.B. 300 + 17 = 317) ← **zurück zur Mitte!**
- pos=4: `base_h - wiggle` (z.B. 300 - 35 = 265)

Das Servo fuhr also **zurück** von 335 auf 317, dann **vorwärts** auf 265.
→ Sichtbares **Zucken**!

Mit der neuen Implementierung:
- pos=2: 335
- pos=3: 300 ← **gleichmäßig in Richtung pos=4**
- pos=4: 265
→ **Fließende Bewegung**

---

## Verwandte Änderungen

Siehe auch:
- `ENHANCEMENT_SMOOTH_SERVOS_2026-01-19.md` - Smooth-Modus Implementierung
- `REFACTORING_LEG_FUNCTIONS_2026-01-19.md` - Leg control refactoring
- `BUGFIX_JERK_DIRECTION_2026-01-19.md` - Frühere Jerk-Probleme

---

## Status

✅ **Behoben** - Smooth-Modus funktioniert jetzt ohne Zuckbewegungen
