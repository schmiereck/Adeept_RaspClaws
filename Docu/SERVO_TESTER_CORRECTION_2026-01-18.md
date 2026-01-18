# ServoTester - Korrektur der Servo-Zuordnung

**Datum**: 2026-01-18  
**Problem**: Rechte Seite Servos vertauscht (vorne/hinten) + Head Servos vertauscht  
**Status**: ✅ Behoben

---

## Problem 1: Rechte Seite (Beine)

Die Servo-Zuordnung im ServoTester war für die **rechte Seite falsch**:

**Vorher (FALSCH)**:
- Right I (Ch 6-7) → Front-Right ❌
- Right II (Ch 8-9) → Mid-Right ✓
- Right III (Ch 10-11) → Back-Right ❌

**Tatsächlich (RICHTIG)**:
- Right I (Ch 6-7) → **Back-Right** ✓
- Right II (Ch 8-9) → Mid-Right ✓
- Right III (Ch 10-11) → **Front-Right** ✓

---

## Problem 2: Head (Kopf)

Die Servo-Zuordnung für den **Head war ebenfalls vertauscht**:

**Vorher (FALSCH)**:
- Up/Down → Ch 12 ❌
- Left/Right → Ch 13 ❌

**Tatsächlich (RICHTIG)**:
- Up/Down → **Ch 13** ✓
- Left/Right → **Ch 12** ✓

**Verifikation aus Move.py**:
```python
def look_up(wiggle):
    pwm.set_pwm(13, 0, Up_Down_input)    # Ch 13 für Up/Down

def look_down(wiggle):
    pwm.set_pwm(13, 0, Up_Down_input)    # Ch 13 für Up/Down

def look_left(wiggle):
    pwm.set_pwm(12, 0, Left_Right_input) # Ch 12 für Left/Right

def look_right(wiggle):
    pwm.set_pwm(12, 0, Left_Right_input) # Ch 12 für Left/Right
```

---

## Ursache (Beine)

Die Zuordnung basierte auf einer falschen Annahme:
- Links: I=Front, II=Mid, III=Back
- Rechts: I=Front, II=Mid, III=Back (Annahme)

Aber laut **Move.py Zeilen 200-202** ist die Anordnung **diagonal**:

```python
left_I   -<forward>-- right_III
left_II  ---<BODY>---  right_II
left_III -<Backward>-   right_I
```

**Bedeutung**:
- **Front-Left** (left_I) ist diagonal gegenüber von **Front-Right** (right_III)
- **Mid-Left** (left_II) ist gegenüber von **Mid-Right** (right_II)
- **Back-Left** (left_III) ist diagonal gegenüber von **Back-Right** (right_I)

---

## Lösung

### Korrigierte Zuordnung (Beine)

#### Linke Seite (unverändert):
```
Front-Left (Left I)   → Ch 0-1
Mid-Left (Left II)    → Ch 2-3
Back-Left (Left III)  → Ch 4-5
```

#### Rechte Seite (KORRIGIERT):
```
Front-Right (Right III) → Ch 10-11  ← GEÄNDERT
Mid-Right (Right II)    → Ch 8-9    ← Unverändert
Back-Right (Right I)    → Ch 6-7    ← GEÄNDERT
```

### Korrigierte Zuordnung (Head)

```
Up/Down    → Ch 13  ← GEÄNDERT (war Ch 12)
Left/Right → Ch 12  ← GEÄNDERT (war Ch 13)
```

### Diagonale Anordnung

```
      VORNE
   ┌─────────┐
L  │ 0-1  10-11│ R
E  │         │ I
F  │ 2-3   8-9│ G
T  │         │ H
   │ 4-5   6-7│ T
   └─────────┘
     HINTEN
```

**Gegenüberliegende Beine**:
- Ch 0-1 (Front-Left) ↔ Ch 10-11 (Front-Right)
- Ch 2-3 (Mid-Left) ↔ Ch 8-9 (Mid-Right)
- Ch 4-5 (Back-Left) ↔ Ch 6-7 (Back-Right)

---

## Geänderte Dateien

### 1. `Server/ServoTester.py`

**SERVO_LAYOUT Struktur korrigiert**:

```python
# Beine (rechte Seite)
'right': {
    'Front-Right (Right III)': {  # KORRIGIERT
        'Rotation': 10,
        'Height': 11
    },
    'Mid-Right (Right II)': {
        'Rotation': 8,
        'Height': 9
    },
    'Back-Right (Right I)': {  # KORRIGIERT
        'Rotation': 6,
        'Height': 7
    }
},
# Head (Kopf)
'head': {
    'Head': {
        'Up/Down': 13,      # KORRIGIERT (war 12)
        'Left/Right': 12    # KORRIGIERT (war 13)
    }
}
```

### 2. `Docu/SERVO_TESTER_DE.md`

- Kanal-Mapping Tabelle korrigiert (Beine & Head)
- GUI-Layout Diagramm aktualisiert
- Hinweis auf diagonale Anordnung hinzugefügt

### 3. `Docu/SERVO_TESTER_EN.md`

- Channel mapping table corrected (Legs & Head)
- GUI layout diagram updated
- Note about diagonal arrangement added

---

## Warum war es falsch?

### Ursprüngliche Annahme:
"Die Nummerierung I/II/III ist auf beiden Seiten gleich (Front/Mid/Back)"

### Tatsächliche Implementierung:
"Die Nummerierung ist **diagonal gespiegelt** für synchrone Bewegung"

**Grund**: Beim Vorwärtslaufen müssen gegenüberliegende Beine synchron bewegen:
- Wenn Front-Left (I) nach vorne geht, muss Front-Right (III) auch nach vorne
- Die Funktion `right_I()` steuert das **Hinter-Rechts** Bein
- Die Funktion `right_III()` steuert das **Vorne-Rechts** Bein

---

## Verifikation

### Move.py Kommentar:
```python
left_I   -<forward>-- right_III
left_II  ---<BODY>---  right_II
left_III -<Backward>-   right_I
```

### Verwendung in Lauf-Sequenzen:

**Beispiel aus Move.py**:
```python
# Gleichzeitiger Aufruf gegenüberliegender Beine:
left_I(step_I, speed, 0)      # Front-Left
right_III(step_I, speed, 0)   # Front-Right (diagonal gegenüber!)

left_II(step_II, speed, 0)    # Mid-Left  
right_II(step_II, speed, 0)   # Mid-Right

left_III(step_I, -speed, 0)   # Back-Left
right_I(step_I, -speed, 0)    # Back-Right (diagonal gegenüber!)
```

---

## Ergebnis

✅ **ServoTester zeigt jetzt korrekte Positionen an**:
- **Beine**: Front-Right ist wirklich vorne (Ch 10-11), Back-Right ist wirklich hinten (Ch 6-7)
- **Head**: Up/Down verwendet Ch 13, Left/Right verwendet Ch 12

✅ **Übereinstimmung mit Move.py**:
- Die Bein-Gruppen entsprechen den Funktionen in Move.py
- Diagonale Anordnung ist korrekt dargestellt
- Head-Funktionen (`look_up`, `look_down`, `look_left`, `look_right`) stimmen überein

✅ **Keine Hardware-Änderung nötig**:
- Die Servos sind korrekt angeschlossen
- Nur die Software-Beschreibung war falsch

---

## Wichtiger Hinweis

**⚠️ Die Nummerierung I/II/III ist bewusst anders!**

Dies ist **kein Fehler**, sondern eine **Design-Entscheidung** für:
1. **Synchrone Bewegung**: Gegenüberliegende Beine arbeiten zusammen
2. **Code-Struktur**: `left_I()` und `right_I()` bewegen **verschiedene** Positionen
3. **Diagonale Geometrie**: Roboter hat diagonale Bein-Paare

**Merksatz**: 
"Left-I ist diagonal gegenüber von Right-III"

---

**Version**: 2.2 (Korrektur)  
**Status**: ✅ Behoben & Dokumentiert
