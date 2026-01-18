# ServoTester - Korrektur der Servo-Zuordnung

**Datum**: 2026-01-18  
**Problem**: Rechte Seite Servos vertauscht (vorne/hinten)  
**Status**: ✅ Behoben

---

## Problem

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

## Ursache

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

### Korrigierte Zuordnung

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
}
```

### 2. `Docu/SERVO_TESTER_DE.md`

- Kanal-Mapping Tabelle korrigiert
- GUI-Layout Diagramm aktualisiert
- Hinweis auf diagonale Anordnung hinzugefügt

### 3. `Docu/SERVO_TESTER_EN.md`

- Channel mapping table corrected
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
- Front-Right ist wirklich vorne (Ch 10-11)
- Back-Right ist wirklich hinten (Ch 6-7)

✅ **Übereinstimmung mit Move.py**:
- Die Bein-Gruppen entsprechen den Funktionen in Move.py
- Diagonale Anordnung ist korrekt dargestellt

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
