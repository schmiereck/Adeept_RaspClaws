# Bugfix: Zucken im Smooth-Modus behoben

**Datum:** 2026-01-19  
**Problem:** Im Smooth-Modus (slow) zucken die Beine während der Luftphase zurück zur Mitte  
**Ursache:** Position 3 im Bewegungszyklus war falsch implementiert

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
