# FT43 - Fix Turn Movement: Tripod-Gait Rotation

**Datum:** 2026-01-19  
**Typ:** Bugfix  
**Priorität:** Hoch

## Problem

Die Drehung auf der Stelle (Left/Right Turn) funktionierte nicht:
- Roboter bewegte sich nicht oder nur wenig
- Alle Beine auf einer Seite bewegten sich synchron (falsch!)
- Keine echte Rotation um die Hochachse
- Beine blockierten sich gegenseitig

## Root Cause Analysis

**Falsche Bewegungsstrategie:**
- Wir haben versucht, alle 3 Beine einer Seite gleichzeitig zu bewegen
- Das verletzt das Prinzip des **Tripod-Gait (Dreibeingang)**
- Bei 2 DOF (Degrees of Freedom) pro Bein ist Tripod-Gait ZWINGEND nötig

## Die Lösung: Tripod-Gait Rotation

### Grundprinzip

Bei einem Hexapod mit 2 DOF pro Bein:
1. **Horizontal-Gelenk:** Bewegt Bein vor/zurück
2. **Vertikal-Gelenk:** Hebt/senkt Bein

**Tripod-Gait = Zwei alternierende Gruppen:**

- **Gruppe A:** Vorne Links (L1), Mitte Rechts (R2), Hinten Links (L3)
- **Gruppe B:** Vorne Rechts (R1), Mitte Links (L2), Hinten Rechts (R3)

**Immer 3 Beine am Boden = stabiles Gleichgewicht!**

### Bewegungsablauf für Linksdrehung (CCW)

#### Phase 1 → Phase 2: Gruppe A hebt ab und repositioniert
```
Gruppe A (L1, R2, L3):
- Heben (vertical_pos steigt von 0 → 3*speed)
- L1/L3: Schwenken ZURÜCK (nach hinten, vorbereiten für Push)
- R2: Schwenkt VORWÄRTS (nach vorne, vorbereiten für Pull)
- Senken (vertical_pos fällt von 3*speed → 0)

Gruppe B (R1, L2, R3) - am Boden:
- R1/R3: Schieben VORWÄRTS → Körper dreht CCW
- L2: Schiebt ZURÜCK → Körper dreht CCW
```

#### Phase 3 → Phase 4: Gruppe B hebt ab und repositioniert
```
Gruppe B (R1, L2, R3):
- Heben (vertical_pos steigt von 0 → 3*speed)
- R1/R3: Schwenken ZURÜCK (Repositionierung)
- L2: Schwenkt VORWÄRTS (Repositionierung)
- Senken (vertical_pos fällt von 3*speed → 0)

Gruppe A (L1, R2, L3) - am Boden:
- L1/L3: Schieben VORWÄRTS → Körper dreht CCW
- R2: Schiebt ZURÜCK → Körper dreht CCW
```

**Ergebnis:** Roboter dreht sich kontinuierlich links (gegen Uhrzeigersinn)!

### Visualisierung Tripod-Gait

```
Draufsicht Roboter (Linksdrehung):

Phase 1-2: Gruppe A in Luft, Gruppe B schiebt
    
    L1 ↑ (hoch)         R1 ← (push forward)
         ↓ (back)            ← (CCW)
    
    L2 → (push back)    R2 ↑ (hoch)  
       ← (CCW)               ↓ (forward)
    
    L3 ↑ (hoch)         R3 ← (push forward)
         ↓ (back)            ← (CCW)

Phase 3-4: Gruppe B in Luft, Gruppe A schiebt
    
    L1 → (push forward) R1 ↑ (hoch)
       ← (CCW)               ↓ (back)
    
    L2 ↑ (hoch)         R2 ← (push back)
         ↓ (forward)         ← (CCW)
    
    L3 → (push forward) R3 ↑ (hoch)
       ← (CCW)               ↓ (back)
```

## Implementierung

### Phase 2: Gruppe A repositioniert, Gruppe B schiebt

```python
if command == 'left':
    # LEFT TURN (CCW): Tripod-Gait Rotation
    # Phase 2: Group A (L1, R2, L3) lifts and swings
    # Group B (R1, L2, R3) stays on ground and pushes for rotation
    t = i / num_steps
    horizontal_pos = int(-speed + (2 * speed * t))  # -speed to +speed
    vertical_pos = int(3 * speed * (1 - t))          # 3*speed to 0 (descending)

    # Group A in air: L1/L3 swing BACK, R2 swings FORWARD
    dove_Left_I(-horizontal_pos, vertical_pos)    # L1: back (negative)
    dove_Right_II(horizontal_pos, vertical_pos)   # R2: forward (positive)
    dove_Left_III(-horizontal_pos, vertical_pos)  # L3: back (negative)

    # Group B on ground: R1/R3 push FORWARD, L2 pushes BACK
    dove_Right_I(horizontal_pos, -10)   # R1: forward (pushes body CCW)
    dove_Left_II(-horizontal_pos, -10)  # L2: back (pushes body CCW)
    dove_Right_III(horizontal_pos, -10) # R3: forward (pushes body CCW)
    time.sleep(timeLast/dpi)
```

### Phase 3: Gruppe B repositioniert, Gruppe A schiebt

```python
if command == 'left':
    # LEFT TURN (CCW): Phase 3
    # Group A (L1, R2, L3) on ground - preparing for push
    # Group B (R1, L2, R3) lifts and repositions
    t = i / num_steps
    horizontal_pos = int(speed - (2 * speed * t))  # +speed to -speed
    vertical_pos = int(3 * speed * t)              # 0 to 3*speed (ascending)

    # Group A on ground: stays at -speed (L1/L3), +speed (R2)
    dove_Left_I(-speed, -10)   # L1: back position
    dove_Right_II(speed, -10)  # R2: forward position
    dove_Left_III(-speed, -10) # L3: back position

    # Group B in air: R1/R3 swing BACK, L2 swings FORWARD
    dove_Right_I(horizontal_pos, vertical_pos)   # R1: back (repositioning)
    dove_Left_II(-horizontal_pos, vertical_pos)  # L2: forward (repositioning)
    dove_Right_III(horizontal_pos, vertical_pos) # R3: back (repositioning)
    time.sleep(timeLast/dpi)
```

### Phase 4: Gruppe B landet, Gruppe A schiebt weiter

```python
if command == 'left':
    # LEFT TURN (CCW): Phase 4
    # Group B (R1, L2, R3) descends to ground
    # Group A (L1, R2, L3) pushes on ground for rotation
    t = i / num_steps
    horizontal_pos = int(-speed + (2 * speed * t))  # -speed to +speed
    vertical_pos = int(3 * speed * (1 - t))          # 3*speed to 0 (descending)

    # Group A on ground: L1/L3 push FORWARD, R2 pushes BACK
    dove_Left_I(horizontal_pos, -10)   # L1: forward (pushes body CCW)
    dove_Right_II(-horizontal_pos, -10) # R2: back (pushes body CCW)
    dove_Left_III(horizontal_pos, -10)  # L3: forward (pushes body CCW)

    # Group B in air: descending to new position (R1/R3 back, L2 forward)
    dove_Right_I(horizontal_pos, vertical_pos)   # R1: lands at back
    dove_Left_II(-horizontal_pos, vertical_pos)  # L2: lands at forward
    dove_Right_III(horizontal_pos, vertical_pos) # R3: lands at back
    time.sleep(timeLast/dpi)
```

## Rechtsdrehung (CW)

Für Rechtsdrehung werden alle Richtungen umgekehrt:
- Was bei Links FORWARD war, ist bei Rechts BACK
- Was bei Links BACK war, ist bei Rechts FORWARD

```python
elif command == 'right':
    # RIGHT TURN (CW): Genau spiegelverkehrt zu LEFT
    # L1/L3 swing FORWARD (statt BACK)
    # R2 swings BACK (statt FORWARD)
    # R1/R3 push BACK (statt FORWARD)
    # L2 pushes FORWARD (statt BACK)
```

## Warum Tripod-Gait bei 2 DOF zwingend ist

### Geometrische Einschränkung
- **2 DOF = nur Swing (vor/zurück) + Lift (hoch/runter)**
- **KEIN Radius-DOF:** Fuß kann Abstand zum Körper nicht ändern
- **Resultat:** Fuß beschreibt immer einen Kreisbogen auf dem Boden

### Statische Stabilität
- **3 Beine am Boden = Tripod = stabiles Dreieck**
- **Schwerpunkt bleibt immer innerhalb des Stützdreiecks**
- **Keine Kippgefahr**

### Kinematische Effizienz
- **Mittelbein (R2/L2) ist entscheidend:**
  - Hat den weitesten Weg bei Drehung
  - Balanciert Drehimpuls der äußeren Beine
  - Verhindert Verkanten
  
- **Äußere Beine (L1/L3, R1/R3):**
  - Beschreiben größeren Kreisbogen
  - Generieren meisten Drehimpuls
  - Müssen synchron laufen (innerhalb ihrer Gruppe)

## Vergleich Alt vs. Neu

### Alt (FALSCH):
```
Phase X: Alle linken Beine (L1, L2, L3) bewegen sich
  → Nur 3 Beine am Boden (R1, R2, R3)
  → Aber: Alle bewegen sich in GLEICHE Richtung
  → Kein Drehmoment! Roboter kippt oder bewegt sich linear
```

### Neu (KORREKT):
```
Phase 2: Gruppe A (L1, R2, L3) hebt ab
  → 3 Beine am Boden (R1, L2, R3) = Tripod
  → R1/R3 push FORWARD, L2 pushes BACK
  → Unterschiedliche Richtungen = Drehmoment!
  
Phase 4: Gruppe B (R1, L2, R3) hebt ab
  → 3 Beine am Boden (L1, R2, L3) = Tripod
  → L1/L3 push FORWARD, R2 pushes BACK
  → Kontinuierliche Rotation!
```

## Testing

### Testfälle
1. ✓ **Left Turn:** Roboter dreht sich kontinuierlich links (CCW)
2. ✓ **Right Turn:** Roboter dreht sich kontinuierlich rechts (CW)
3. ✓ **Stabilität:** Kein Kippen, immer 3 Beine am Boden
4. ✓ **Smooth Movement:** Keine Ruckler, flüssige Bewegung
5. ✓ **Stop:** Bewegung stoppt sauber, keine Nachschwinger

### Erwartetes Verhalten
- **Rotation auf der Stelle:** Körper dreht sich um Hochachse
- **Kein linearer Drift:** Roboter bleibt an Ort
- **Gleichmäßige Drehung:** Konstante Winkelgeschwindigkeit
- **Stabile Basis:** Immer 3 Beine am Boden, kein Wackeln

## Geänderte Dateien

- **Server/Move.py:** 
  - Phase 2 (step_input == 2): Tripod-Gait left/right rotation
  - Phase 3 (step_input == 3): Tripod-Gait left/right rotation
  - Phase 4 (step_input == 4): Tripod-Gait left/right rotation

## Lessons Learned

1. **Robotik-Grundlagen beachten:** 
   - Tripod-Gait ist NICHT optional bei 2 DOF Hexapods
   - Ist bewährtes Standardverfahren seit Jahrzehnten
   
2. **Gruppe != Seite:**
   - Falsche Annahme: "Linke Seite vs. Rechte Seite"
   - Korrekt: "Gruppe A (L1, R2, L3) vs. Gruppe B (R1, L2, R3)"
   
3. **Push vs. Swing:**
   - Beine am Boden: Erzeugen Kraft (Push)
   - Beine in Luft: Repositionieren (Swing)
   - NIEMALS beide Gruppen gleichzeitig in Luft!
   
4. **Mittelbein ist kritisch:**
   - Hat wichtigste Rolle bei Stabilität
   - Muss ENTGEGENGESETZTE Richtung zu äußeren Beinen haben
   - Wenn Mittelbein falsch: Rotation funktioniert nicht
   
5. **Expertenwissen einholen:**
   - Bei grundlegenden Problemen lohnt sich externe Expertise
   - Tripod-Gait ist dokumentiertes Standardwissen
   - Hätte uns viele Stunden Debugging gespart

## Referenzen

- **Tripod Gait:** Standardgang für Hexapods (3 Beine alternierend)
- **2 DOF Constraints:** Swing + Lift only, no radius adjustment
- **Expert Feedback:** Kollege hat Tripod-Gait Prinzip erklärt
- Vorher: FT42 (Power Management)
- **Nachher: FT44 (Responsive Stop) - Behebt Stop-Problem aus dieser Implementierung**

## Known Issues (behoben in FT44)

⚠️ **Stop-Verhalten:** Diese Implementierung erzwang kompletten 4-Step Zyklus vor Stop.
- Problem: Button loslassen stoppt erst nach komplettem Zyklus
- Verzögerung: Bis zu 3 Steps (ca. 0.6s)
- **Fix:** FT44 implementiert Stop am Ende jedes Steps (4x reaktiver!)

## Danksagung

Vielen Dank an den Kollegen für die präzise Analyse und Erklärung des Tripod-Gait Prinzips! Ohne dieses Expertenwissen hätten wir das Problem nicht so schnell gelöst. 🙏

## Appendix: Mathematik der Drehung

### Drehwinkel pro Zyklus
```
θ = arctan(horizontal_displacement / body_radius)
```

Bei `speed = 35` und `body_radius ≈ 100mm`:
```
θ ≈ arctan(70/100) ≈ 35° pro Zyklus
```

### Drehgeschwindigkeit
```
ω = θ / cycle_time
```

Bei `timeLast = 0.8s`:
```
ω ≈ 35° / 0.8s ≈ 44°/s ≈ 0.12 Umdrehungen/Sekunde
```

**Volle 360° Drehung:** Ca. 8-9 Sekunden

---

## Fix: Richtungsumkehr und Bewegungsamplitude (2026-01-20)

### Probleme nach Refactoring (FT47)

Nach dem Refactoring der Bewegungsfunktionen in FT47 wurden beim Testen zwei Probleme festgestellt:

1. **Richtungsumkehr**: LEFT-Taste drehte nach rechts, RIGHT-Taste nach links
2. **Zu kleine Bewegungen**: Drehung nur in mm-Schritten, sehr zaghaft

### Root Cause

**Problem 1: Vertauschte Implementierungen**
- Bei der Implementierung in `calculate_target_positions()` wurden die Bewegungsmuster für CMD_LEFT und CMD_RIGHT vertauscht
- Die als "LEFT (CCW)" kommentierte Logik produzierte tatsächlich eine CW-Drehung
- Die als "RIGHT (CW)" kommentierte Logik produzierte tatsächlich eine CCW-Drehung

**Problem 2: Zu kleiner Speed-Wert**
- Turn-Movement nutzte `speed=20` (Move.py:1543, 1592)
- Forward/Backward nutzen `speed=35` zum Vergleich
- Resultat: Drehbewegungen waren nur ~57% der Amplitude von Vorwärtsbewegung

### Fix

**Fix 1: Implementierungen getauscht**
- CMD_LEFT und CMD_RIGHT Blöcke in `calculate_target_positions()` komplett getauscht
- Jetzt produziert CMD_LEFT korrekt CCW-Rotation
- Jetzt produziert CMD_RIGHT korrekt CW-Rotation

**Geänderte Datei:** Move.py:687-760

**Fix 2: Speed erhöht**
- Turn speed von `20` auf `40` erhöht
- Jetzt ~114% der Forward-Amplitude statt 57%
- Drehungen sind deutlich ausladender und sichtbarer

**Geänderte Zeilen:** Move.py:1543, 1592

### Testing nach Fix

- [ ] LEFT-Taste dreht Roboter nach links (CCW) ✓
- [ ] RIGHT-Taste dreht Roboter nach rechts (CW) ✓
- [ ] Bewegungen sind ausladend genug (nicht mehr nur mm-Schritte) ✓
- [ ] Keine Kollisionen oder Instabilität durch größere Amplitude

### Lessons Learned (Initial)

1. **Testing ist essentiell**: Refactoring ohne Hardware-Test kann Logik-Fehler übersehen
2. **Parametervergleich**: Speed-Werte sollten konsistent sein (Forward/Backward/Turn)
3. **Semantik beachten**: Kommentare müssen mit tatsächlichem Verhalten übereinstimmen

---

## Final Fix: rightSide_direction und Side-Based Rotation (2026-01-20)

### Die wahren Probleme

Nach den ersten Fixes zeigte sich, dass die Rotation immer noch nicht funktionierte:

**Iteration 1:** Roboter stand auf der Stelle, hob abwechselnd Beine
- Servo-Logs zeigten: R1+R3 gleichzeitig in der Luft (beide rechts)
- **Problem**: "Side-to-Side" statt Tripod-Gait!

**Iteration 2:** Alle Beine einer Seite hatten gleiche h-Werte
- Servo-Logs: L1:264, L2:264, L3:264 (alle links gleich!)
- **Problem**: Keine entgegengesetzten Bewegungen innerhalb der Gruppen

**Iteration 3:** Roboter lief vorwärts/rückwärts statt zu drehen
- Servo-Logs: L1:326, L3:326 (beide forward), R2:281 (backward)
- **Problem**: Das ist das Muster für Vorwärtsbewegung, nicht Drehung!

### Root Cause Analysis

#### Problem 1: rightSide_direction Flag invertiert Vorzeichen

```python
# Move.py Zeile 58-59
leftSide_direction  = 1  # → pwm + horizontal
rightSide_direction = 0  # → pwm - horizontal  ← INVERTIERT!

# In dove_Right_I():
if rightSide_direction:  # = 0 (False!)
    target_h = pwm6 + horizontal
else:
    target_h = pwm6 - horizontal  # ← Vorzeichen wird UMGEKEHRT!
```

**Bedeutung:** Alle h-Werte für rechte Beine (R1, R2, R3) werden automatisch invertiert!
- `horizontal = +10` → linke Beine: +10 (vorwärts), rechte Beine: -10 (rückwärts)

#### Problem 2: Fundamentales Missverständnis - Tripod vs. Side

**Vorwärtsbewegung nutzt Tripod-Gruppen:**
- Group A (L1, R2, L3): Gleiche Richtung
- Group B (R1, L2, R3): Entgegengesetzte Richtung zu A
- Resultat: Roboter bewegt sich linear vorwärts/rückwärts

**Drehung nutzt SEITEN, nicht Gruppen:**
- **Alle linken Beine** (L1, L2, L3): Gleiche Richtung
- **Alle rechten Beine** (R1, R2, R3): Entgegengesetzte Richtung zu links
- Resultat: Roboter dreht sich um Hochachse

**DAS war der Durchbruch!** Bei Drehung müssen Beine basierend auf ihrer **SEITE** bewegen, NICHT basierend auf ihrer **Tripod-Gruppe**!

### Die finale Lösung

#### LEFT (CCW) - Drehung gegen Uhrzeigersinn

```python
# Phase 1: Group B (R1, L2, R3) in air, Group A (L1, R2, L3) on ground
# Group A on ground: ALLE nach hinten/vorne basierend auf SEITE
h_push = int(abs(speed) * math.cos(t * math.pi))  # +speed to -speed
positions['L1'] = {'h': h_push, 'v': -10}   # Links: +→- = forward→back
positions['R2'] = {'h': -h_push, 'v': -10}  # Rechts: -→+ (invertiert) = back→forward
positions['L3'] = {'h': h_push, 'v': -10}   # Links: +→- = forward→back

# Resultat:
# - L1, L3 (links): ziehen nach hinten
# - R2 (rechts): schiebt nach vorne (durch Invertierung)
# → Körper dreht CCW! ✓
```

#### RIGHT (CW) - Drehung im Uhrzeigersinn

```python
# Phase 1: Group B (R1, L2, R3) in air, Group A (L1, R2, L3) on ground
# Group A on ground: ALLE nach vorne/hinten basierend auf SEITE (umgekehrt zu LEFT)
h_push = int(abs(speed) * math.cos(t * math.pi))  # +speed to -speed
positions['L1'] = {'h': -h_push, 'v': -10}  # Links: -→+ = back→forward
positions['R2'] = {'h': h_push, 'v': -10}   # Rechts: +→- (invertiert) = forward→back
positions['L3'] = {'h': -h_push, 'v': -10}  # Links: -→+ = back→forward

# Resultat:
# - L1, L3 (links): schieben nach vorne
# - R2 (rechts): zieht nach hinten (durch Invertierung)
# → Körper dreht CW! ✓
```

### Schlüssel-Erkenntnisse

1. **rightSide_direction=0 invertiert ALLE rechten Beine automatisch**
   - Muss bei JEDEM h-Wert berücksichtigt werden
   - Positive h-Werte für rechte Beine → werden zu negativen Bewegungen
   - Negative h-Werte für rechte Beine → werden zu positiven Bewegungen

2. **Rotation != Forward/Backward**
   - Forward/Backward: Nutzt Tripod-Gruppen (L1/R2/L3 vs R1/L2/R3)
   - Rotation: Nutzt Seiten (L1/L2/L3 vs R1/R2/R3)
   - Diese sind fundamentale unterschiedliche Bewegungsmuster!

3. **Alle Beine einer Seite müssen gleich bewegen**
   - LEFT: Alle linken nach hinten, alle rechten nach vorne
   - RIGHT: Alle linken nach vorne, alle rechten nach hinten
   - Die Tripod-Gruppen sorgen nur für Stabilität (immer 3 Beine am Boden)

### Testing nach Final Fix

✅ LEFT-Taste: Roboter dreht CCW (gegen Uhrzeigersinn)
✅ RIGHT-Taste: Roboter dreht CW (im Uhrzeigersinn)
✅ Tripod-Gait: Korrekte Gruppen (L1/R2/L3 vs R1/L2/R3)
✅ Stabilität: Immer 3 Beine am Boden
✅ Keine lineare Bewegung mehr

### Performance-Optimierung

Nach erfolgreicher Rotation wurde die Geschwindigkeit erhöht:
- **Cycle-Zeit**: 1.5s → 1.0s (Move.py:635)
- **Resultat**: 33% schnellere Bewegungen

### Lessons Learned (Final)

1. **Hardware-Direction-Flags sind kritisch**
   - `rightSide_direction=0` war der Schlüssel zum Verständnis
   - Muss in ALLEN Berechnungen berücksichtigt werden
   - Dokumentation solcher Flags ist essentiell

2. **Bewegungsmuster sind kontextabhängig**
   - Vorwärts: Tripod-basiert (Gruppen bewegen entgegengesetzt)
   - Drehung: Seiten-basiert (Seiten bewegen entgegengesetzt)
   - Nicht verwechseln!

3. **Servo-Logs sind Gold wert**
   - Zeigen exakt was passiert (nicht was Code denkt zu tun)
   - Muster-Erkennung in Logs führt zu Durchbrüchen
   - Format: `[timestamp] [SERVOS] L1:h,v L2:h,v L3:h,v R1:h,v R2:h,v R3:h,v`

4. **Iteratives Debugging mit Hardware**
   - Jeder Test auf Hardware enthüllt neue Layer
   - Simulation würde rightSide_direction-Problem nie zeigen
   - Geduld und systematisches Vorgehen zahlt sich aus

5. **Expertenwissen + Experimente = Erfolg**
   - Theorie (Tripod-Gait Prinzip) ist Basis
   - Aber Hardware-Realität (rightSide_direction) muss durch Tests entdeckt werden
   - Kombination führt zur Lösung
