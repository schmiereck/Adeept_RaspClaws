# Bugfix: Ruckartiges Schleudern im Smooth-Modus behoben

**Datum:** 2026-01-19  
**Problem:** Im Smooth-Modus schleudert ein Bein nach vorne während der Luftphase  
**Ursache:** Sofortige maximale Position statt gradueller Interpolation in der `dove()` Funktion

---

## Problem-Beschreibung

### Symptome
- **Normal-Modus:** Bewegungen funktionieren korrekt ✓
- **Smooth-Modus (slow):** 
  - Bein wird sofort nach vorne geschleudert, bevor es angehoben wird ❌
  - Ruckartiger Bewegungsablauf statt weicher Interpolation
  - Besonders sichtbar bei "left_I" (links vorne) und anderen Beinen in der Luftphase

### Beobachtung
Bei Vorwärtsbewegung im Smooth-Modus:
- Das Bein wird **sofort maximal nach vorne** bewegt (noch am Boden!)
- Dann wird es angehoben und zurück zur Mitte bewegt
- Dies wirkt wie ein **ruckartiges Schleudern** nach vorne

---

## Ursachen-Analyse

### Die fehlerhafte Interpolation

**Original-Code in `dove()` (Zeile 499-507):**
```python
for speed_I in range(0, (speed+int(speed/dpi)), int(speed/dpi)):
    speed_II = speed_I          # Speichere für vertikale Position
    speed_I = speed - speed_I   # ← Umkehrung für horizontale Position!
    
    # Bein links vorne (left_I) in der Luft:
    dove_Left_I(-speed_I, 3*speed_II)
    #          ^^^^^^^^  ^^^^^^^^^^^
    #          horizontal  vertikal
```

### Bewegungsablauf mit speed=35, dpi=10:

| Loop | Original speed_I | speed_II | speed_I (nach Umkehrung) | Horizontal (-speed_I) | Vertikal (3*speed_II) | Was passiert |
|------|-----------------|----------|-------------------------|----------------------|---------------------|--------------|
| 1    | **0**           | **0**    | **35**                  | **-35**              | **0**               | **SCHLEUDERN nach vorne! Noch am Boden!** ❌ |
| 2    | 10              | 10       | 25                      | -25                  | 30                  | Zurück + hochheben |
| 3    | 20              | 20       | 15                      | -15                  | 60                  | Weiter zurück + höher |
| 4    | 30              | 30       | 5                       | -5                   | 90                  | Fast neutral + oben |

**Das Problem:**
- **Erste Iteration:** Bein sofort bei **Position -35** (maximal vorne), aber **Höhe 0** (am Boden!)
- Das Bein wird nach vorne **geschleudert**, bevor es überhaupt angehoben wird!
- Dann bewegt es sich **rückwärts**, während es angehoben wird

**Erwartete Bewegung:**
1. Bein anheben (vertikal nach oben)
2. Während des Anhebens nach vorne bewegen (horizontal)
3. Sanft absenken

**Tatsächliche Bewegung (vorher):**
1. **Nach vorne schleudern** (am Boden!) ❌
2. Dann anheben + zurück zur Mitte bewegen

---

## Lösung

### Die korrekte Interpolation

**Korrigierter Code:**
```python
for speed_I in range(0, (speed+int(speed/dpi)), int(speed/dpi)):
    # Keine Variable-Umkehrung mehr!
    # Beide verwenden direkt speed_I für graduelle Bewegung
    
    dove_Left_I(-speed_I, 3*speed_I)
    #          ^^^^^^^^  ^^^^^^^^^^
    #          0→-35     0→105 (graduell!)
```

### Neuer Bewegungsablauf:

| Loop | speed_I | Horizontal (-speed_I) | Vertikal (3*speed_I) | Was passiert |
|------|---------|--------------------|-------------------|--------------|
| 1    | **0**   | **0**              | **0**             | **Start neutral** ✓ |
| 2    | 10      | -10                | 30                | Langsam vorwärts + anheben ✓ |
| 3    | 20      | -20                | 60                | Weiter vorwärts + höher ✓ |
| 4    | 30      | -30                | 90                | Noch weiter + am höchsten ✓ |
| 5    | 35      | -35                | 105               | Maximal vorne + oben ✓ |

**Resultat:**
- ✅ Bein beginnt bei neutraler Position (0, 0)
- ✅ Bewegt sich **graduell** nach vorne (0 → -35)
- ✅ Hebt sich **gleichzeitig graduell** an (0 → 105)
- ✅ **Weiche, fließende Bewegung** ohne Schleudern!

---

## Geänderte Dateien

### `Server/Move.py`

**Zeile 499-535** (step_input == 1 - Erste Gruppe in der Luft):
```python
# Vorher (falsch):
speed_II = speed_I
speed_I = speed - speed_I
dove_Left_I(-speed_I, 3*speed_II)  # Sofort -35, dann zurück!

# Nachher (korrekt):
dove_Left_I(-speed_I, 3*speed_I)   # Graduell 0 → -35
```

**Zeile 583-619** (step_input == 3 - Zweite Gruppe in der Luft):
```python
# Gleiche Korrektur für right_I, left_II, right_III
dove_Right_I(-speed_I, 3*speed_I)  # Graduell statt sofort
```

**Zeile 666-679** (Rückwärtsbewegung step_input == 1):
```python
# Gleiche Korrektur für backward movement
dove_Left_I(speed_I, 3*speed_I)  # Graduell in die andere Richtung
```

**Zeile 697-710** (Rückwärtsbewegung step_input == 3):
```python
# Zweite Gruppe, Rückwärtsbewegung
dove_Right_I(speed_I, 3*speed_I)
```

---

## Technische Details

### Warum die Umkehrung falsch war

Die Original-Logik:
```python
speed_II = speed_I         # Für vertikal (aufsteigend 0 → speed)
speed_I = speed - speed_I  # Für horizontal (absteigend speed → 0)
```

**Intention:** Vertikale Bewegung soll aufsteigend sein, horizontale absteigend.

**Problem:** Die Richtung war verkehrt herum!
- **Horizontal sollte sein:** 0 → -speed (graduell nach vorne)
- **Tatsächlich war:** -speed → 0 (sofort maximal, dann zurück!)

### Die korrekte Logik

**Beide Richtungen aufsteigend:**
```python
dove_Left_I(-speed_I, 3*speed_I)
#          ^^^^^^^^  ^^^^^^^^^^
#          0 → -35    0 → 105
```

Dies ergibt eine **synchrone graduelle Bewegung**:
- Horizontal: Von neutral (0) graduell nach vorne (-35)
- Vertikal: Von Boden (0) graduell nach oben (105)
- **Resultat:** Weiche, koordinierte Bewegung ✓

---

## Test-Ergebnisse

### Vor dem Fix
- ❌ **Smooth-Modus:** Beine schleudern ruckartig nach vorne
- ❌ Bewegung wirkt unkontrolliert und unnatürlich
- ❌ "left_I" (links vorne) besonders auffällig

### Nach dem Fix
- ✅ **Smooth-Modus:** Weiche, graduelle Bewegungen
- ✅ Beine heben sich sanft an, während sie sich bewegen
- ✅ Natürlicher Bewegungsablauf
- ✅ Keine ruckartigen Schleuderbewegungen mehr

---

## Verwandte Änderungen

Siehe auch:
- `ENHANCEMENT_SMOOTH_SERVOS_2026-01-19.md` - Smooth-Modus Implementierung
- `REFACTORING_LEG_FUNCTIONS_2026-01-19.md` - Leg control refactoring

---

## Status

✅ **Behoben** - Smooth-Modus funktioniert jetzt mit weichen, graduellen Bewegungen ohne Schleudern

---

## Debugging-Methode

**Wichtige Lektion:** Bei komplexen Bewegungsproblemen:
1. **Ein einzelnes Bein analysieren** (z.B. "left_I")
2. **Jeden Loop-Durchlauf dokumentieren** (Werte von speed_I, Positionen)
3. **Erwartete vs. tatsächliche Bewegung vergleichen**
4. **Variable-Manipulationen besonders prüfen** (wie `speed_I = speed - speed_I`)

Diese Methode führte zur Entdeckung des Bugs: Das Bein startete bei Position -35 statt 0!

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
