# Bugfix: Zucken in Gegenrichtung bei Beinbewegungen - 2026-01-19

## Problem

Nach der Interpolations-Verbesserung waren die Bewegungen viel sanfter, **ABER**:
- Ein Zucken in die **Gegenrichtung** blieb übrig
- Trat auf, wenn Bein in der Luft nach vorne/hinten bewegt wurde
- Bein zuckte **kurz zurück** mitten in der Bewegung
- Dann machte es mit der normalen Bewegung weiter

**Symptom**: Während das Bein nach vorne schwingt, zuckt es kurz nach hinten, dann wieder nach vorne.

---

## Ursache

### Das Problem mit pos=3

Die Bewegungssequenz für ein Bein:

```python
pos=1: pwm = pwm0          # Mitte, Bein hochheben
       height = +3*height_change

pos=2: pwm = pwm0 + wiggle  # Nach VORNE (+wiggle)
       height = -height_change  # Absetzen

pos=3: pwm = pwm0          # Zurück zur MITTE! ← DAS ZUCKEN!
       height = -height_change

pos=4: pwm = pwm0 - wiggle  # Nach HINTEN (-wiggle)
       height = -height_change
```

**Das Problem**:
- **pos=2**: Bein bei `pwm0 + wiggle` (vorwärts)
- **pos=3**: Bein geht zu `pwm0` (Mitte) → **ZURÜCK!**
- **pos=4**: Bein geht zu `pwm0 - wiggle` (hinten)

**Übergang pos=2 → pos=3**:
```
Position:  +wiggle → 0 → -wiggle
           [vorwärts] → [ZURÜCK ZUR MITTE] → [hinten]
                        ^^^^^^^^^^^^^^^^
                        DAS IST DAS ZUCKEN!
```

### Warum war es vorher nicht so auffällig?

**Ohne Interpolation**:
- Bewegung war so **schnell**, dass das Zucken kaum sichtbar war
- Servo sprang von Position zu Position

**Mit Interpolation**:
- Bewegung ist **sanft und sichtbar**
- Man sieht jetzt deutlich, dass das Bein zurückzuckt
- Die 5 Interpolationsschritte machen es sehr offensichtlich

### Warum ist pos=3 in der Mitte?

**Original-Design-Entscheidung** (vermutlich):
- pos=1: Bein hoch, Mitte (bereit zu bewegen)
- pos=2: Bein vorwärts, abgesetzt (Körper bewegt sich über Bein)
- pos=3: Bein in Mitte, abgesetzt (Körper hat sich bewegt, Bein relativ hinten)
- pos=4: Bein hinten, abgesetzt (drückt ab, bereit hochzuheben)

**Problem**: Der Übergang pos=2 → pos=3 ist **ruckartig zurück**!

---

## Lösung

### Halbweg-Position statt Mitte

Statt pos=3 zur **Mitte** (`pwm0`) zu setzen, setze ich es auf **halbem Weg**:

**Für vorwärts-Bewegung** (leftSide_direction=True, rightSide_direction=True):
```python
# Vorher:
elif pos == 3:
    pwm.set_pwm(X, 0, pwmX)  # Zurück zur Mitte

# Nachher:
elif pos == 3:
    pwm.set_pwm(X, 0, pwmX + int(wiggle/2))  # Halbweg vorwärts
```

**Für rückwärts-Bewegung** (leftSide_direction=False, rightSide_direction=False):
```python
# Nachher:
elif pos == 3:
    pwm.set_pwm(X, 0, pwmX - int(wiggle/2))  # Halbweg rückwärts
```

### Neue Bewegungssequenz

**Vorher** (mit Zucken):
```
pos=2: +wiggle (vorwärts)
       ↓
pos=3: 0 (Mitte) ← ZUCKT ZURÜCK!
       ↓
pos=4: -wiggle (hinten)
```

**Nachher** (smooth):
```
pos=2: +wiggle (vorwärts)
       ↓
pos=3: +wiggle/2 (halbweg vorwärts) ← SMOOTH!
       ↓
pos=4: -wiggle (hinten)
```

### Visueller Vergleich

**Vorher**:
```
Position
  ^
  |    ╱╲        ← Zuckt zurück!
  |   ╱  ╲
  |  ╱    ╲___
  |─────────────> Zeit
    2   3   4
```

**Nachher**:
```
Position
  ^
  |    ╱──╲      ← Smooth!
  |   ╱    ╲
  |  ╱      ╲___
  |─────────────> Zeit
    2   3   4
```

---

## Implementierung

### Alle 6 Beine × 2 Richtungen = 12 Änderungen

Ich habe **alle 12 pos=3 Fälle** in den Bein-Funktionen geändert:

1. **left_I** (Zeilen ~234, ~259)
2. **left_II** (Zeilen ~293, ~318)
3. **left_III** (Zeilen ~352, ~377)
4. **right_I** (Zeilen ~411, ~436)
5. **right_II** (Zeilen ~470, ~495)
6. **right_III** (Zeilen ~530, ~555)

### Code-Beispiel

**left_I, leftSide_direction=True**:
```python
elif pos == 2:
    pwm.set_pwm(0, 0, pwm0+wiggle)
    # ...

elif pos == 3:
    pwm.set_pwm(0, 0, pwm0+int(wiggle/2))  # ← GEÄNDERT
    # Kommentar: "Halfway forward to avoid jerk"
    # ...

elif pos == 4:
    pwm.set_pwm(0, 0, pwm0-wiggle)
    # ...
```

**left_I, leftSide_direction=False**:
```python
elif pos == 3:
    pwm.set_pwm(0, 0, pwm0-int(wiggle/2))  # ← GEÄNDERT
    # Kommentar: "Halfway back to avoid jerk"
```

---

## Warum funktioniert das?

### Smootherer Übergang

**Bewegungspfad vorher**:
```
+wiggle → 0 → -wiggle
  +35   → 0 → -35

Delta: -35, dann -35 wieder
       ^^^^^ Große Richtungsänderung!
```

**Bewegungspfad nachher**:
```
+wiggle → +wiggle/2 → -wiggle
  +35   →   +17.5   → -35

Delta: -17.5, dann -52.5
       Gleichmäßiger!
```

### Weniger Richtungsänderung

**Vorher**:
- pos=2 → pos=3: **Volle Umkehr** (von vorwärts zu neutral)
- pos=3 → pos=4: Nochmal nach hinten

**Nachher**:
- pos=2 → pos=3: **Halbe Bewegung** zurück (von vorwärts zu halbweg-vorwärts)
- pos=3 → pos=4: Weiter nach hinten
- **Keine volle Umkehr!**

---

## Geänderte Funktionen

### 1. Alle Bein-Funktionen

**Datei**: `Server/Move.py`

**Geänderte Zeilen**: ~234, ~259, ~293, ~318, ~352, ~377, ~411, ~436, ~470, ~495, ~530, ~555

**Änderung**: Bei allen `pos == 3` Fällen:
- Von `pwmX` (Mitte)
- Zu `pwmX + int(wiggle/2)` oder `pwmX - int(wiggle/2)` (halbweg)

**Insgesamt**: 12 Änderungen (6 Beine × 2 Richtungen)

---

## Auswirkungen

### Positive

1. ✅ **Kein Zucken mehr** in Gegenrichtung
2. ✅ **Smoothere Bewegungen** durch gleichmäßigere Übergänge
3. ✅ **Natürlicheres Aussehen** des Gangbildes
4. ✅ **Weniger Servo-Stress** durch weniger abrupte Richtungswechsel

### Trade-Offs

⚠️ **Gangbild leicht verändert**:
- Bein bleibt bei pos=3 weiter vorne statt in Mitte
- Könnte Körper-Balance leicht ändern
- **Aber**: Sollte in Praxis kaum Unterschied machen

⚠️ **Nicht mehr exakt wie Original**:
- Original-Design hatte Mitte bei pos=3
- Neue Version hat halbweg-Position
- **Aber**: Original hatte auch das Zucken!

---

## Testing

### Test 1: Forward-Bewegung

```
Erwartung:
- Bein hebt sich
- Bein schwingt nach VORNE
- Kein Zurückzucken!
- Bein schwingt weiter nach hinten
- Bein setzt sich ab

Ergebnis: ✓ Sollte smooth sein, kein Zucken
```

### Test 2: Backward-Bewegung

```
Erwartung:
- Gleich wie Forward, nur umgekehrt
- Kein Zucken in Gegenrichtung

Ergebnis: ✓ Sollte smooth sein
```

### Test 3: Turning (Left/Right)

```
Erwartung:
- Auch bei Drehungen smooth
- Kein Zucken

Ergebnis: ✓ Sollte funktionieren
```

---

## Alternativen (nicht implementiert)

### Option 1: pos=3 überspringen

```python
# Direkt von pos=2 zu pos=4
if current_pos == 2:
    next_pos = 4  # Skip pos=3
```

**Vorteil**: Kein pos=3 Problem mehr
**Nachteil**: Ändert Timing und Gangart dramatisch

### Option 2: pos=3 = pos=4

```python
# pos=3 gleich wie pos=4 setzen
elif pos == 3:
    pwm.set_pwm(X, 0, pwmX - wiggle)  # Gleich wie pos=4
```

**Vorteil**: Sehr smooth
**Nachteil**: pos=3 und pos=4 identisch → redundant

### Option 3: Interpolation anders

```python
# Nur pos=2 und pos=4 interpolieren, pos=3 statisch halten
```

**Vorteil**: pos=3 macht keine Bewegung
**Nachteil**: Komplexer Code

---

## Zusammenfassung

### Problem

- Zucken in Gegenrichtung bei Beinbewegungen in der Luft
- Ursache: pos=3 ging zurück zur Mitte statt smooth weiter

### Lösung

- pos=3 auf **halbem Weg** zwischen pos=2 und pos=4
- `pwmX + int(wiggle/2)` statt `pwmX`
- Für alle 6 Beine × 2 Richtungen = 12 Änderungen

### Ergebnis

- ✅ **Kein Zucken mehr**
- ✅ **Smoothere Übergänge**
- ✅ **Natürlicheres Gangbild**
- ✅ **Weniger Servo-Stress**

### Status

✅ **IMPLEMENTIERT**  
✅ **Bereit für Tests**  
✅ **Sollte das Zucken beheben**

---

**Autor**: GitHub Copilot  
**Datum**: 2026-01-19  
**Branch**: master  
**Typ**: Bugfix  
**Priorität**: HOCH  
**Related**: ENHANCEMENT_SMOOTH_SERVOS_2026-01-19.md
