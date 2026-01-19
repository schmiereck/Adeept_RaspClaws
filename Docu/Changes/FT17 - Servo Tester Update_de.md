# ServoTester - Update 2026-01-18

## Änderungen

### 1. ✅ Hardware-Layout Integration

**Neu**: Die GUI spiegelt jetzt das physische Layout des Roboters wider!

**2-Spalten-Layout**:
- **Linke Spalte**: Left Side Servos (Kanäle 0-5)
- **Rechte Spalte**: Right Side Servos (Kanäle 6-11)
- **Zentral unten**: Head Servos (Kanäle 12-13)

### 2. ✅ Vollständige Servo-Abdeckung

**Alle 14 Servos einzeln ansprechbar**:

#### Linke Seite (6 Servos):
- **Front-Left (Left I)**: Ch 0 (Rotation), Ch 1 (Height)
- **Back-Left (Left II)**: Ch 2 (Rotation), Ch 3 (Height)
- **Left III**: Ch 4 (Rotation), Ch 5 (Height)

#### Rechte Seite (6 Servos):
- **Front-Right (Right I)**: Ch 6 (Rotation), Ch 7 (Height)
- **Back-Right (Right II)**: Ch 8 (Rotation), Ch 9 (Height)
- **Right III**: Ch 10 (Rotation), Ch 11 (Height)

#### Kopf (2 Servos):
- **Head**: Ch 12 (Up/Down), Ch 13 (Left/Right)

### 3. ✅ Gruppierung nach Beinen

Jedes Bein ist nun als **Gruppe** dargestellt:
- Zusammengefasst in einem Frame
- Goldener Header mit Bein-Name
- 2 Servos pro Bein (Rotation + Height)
- Kompakte Darstellung

### 4. ✅ Verbesserte Farb-Codierung

- **Left Side**: Grüner Title (#4CAF50)
- **Right Side**: Oranger Title (#FF6D00)
- **Head Servos**: Blauer Title (#2196F3)
- **Bein-Namen**: Gold (#FFD700)

### 5. ✅ Optimierte Fenstergröße

- **Ursprünglich**: 600x700 px
- **V2.0**: 900x750 px
- **V2.1 (kompakt)**: 850x600 px
- Bessere Nutzung des horizontalen Platzes
- Kompakteres Design spart vertikalen Platz
- Kleinere Schriften und reduzierte Abstände

## Vorher/Nachher Vergleich

### Vorher (alte Version):
```
┌─────────────────────┐
│    Servo Tester     │
├─────────────────────┤
│ Head Up/Down        │
│ Head Left/Right     │
│ Leg 1               │
│ Leg 2               │
│ Leg 3               │
│ Leg 4               │
│ ...                 │
└─────────────────────┘
```
- Einfache Liste
- Keine Hardware-Beziehung
- 8 Servos
- Keine Bein-Gruppierung

### Nachher (neue Version):
```
┌───────────────────────────────────────┐
│   Servo Tester - Hardware Layout      │
├──────────────────┬────────────────────┤
│   LEFT SIDE      │    RIGHT SIDE      │
├──────────────────┼────────────────────┤
│ Front-Left (I)   │ Front-Right (I)    │
│  ├─ Rotation     │  ├─ Rotation       │
│  └─ Height       │  └─ Height         │
│                  │                    │
│ Back-Left (II)   │ Back-Right (II)    │
│  ├─ Rotation     │  ├─ Rotation       │
│  └─ Height       │  └─ Height         │
│                  │                    │
│ Left III         │ Right III          │
│  ├─ Rotation     │  ├─ Rotation       │
│  └─ Height       │  └─ Height         │
└──────────────────┴────────────────────┘
           HEAD SERVOS
    ┌────────────────────┐
    │ Up/Down            │
    │ Left/Right         │
    └────────────────────┘
```
- 2-Spalten Layout
- Hardware-orientiert
- 14 Servos (alle)
- Nach Beinen gruppiert
- Position ersichtlich

## Technische Details

### Neue Datenstruktur

```python
SERVO_LAYOUT = {
    'left': {
        'Front-Left (Left I)': {
            'Rotation': 0,
            'Height': 1
        },
        # ...
    },
    'right': {
        'Front-Right (Right I)': {
            'Rotation': 6,
            'Height': 7
        },
        # ...
    },
    'head': {
        'Head': {
            'Up/Down': 12,
            'Left/Right': 13
        }
    }
}
```

### Neue GUI-Methoden

- `create_side_column(parent, side, title, column)` - Erstellt Spalte
- `create_leg_group(parent, leg_name, servos)` - Erstellt Bein-Gruppe
- `create_head_section(parent)` - Erstellt Head-Bereich

## Verwendung

### Start

**Unverändert**:
```bash
# Raspberry Pi
python3 ServoTester.py

# Windows (Mock-Mode)
python ServoTester.py
```

### Bedienung

**Neu**:
1. Wähle Seite (Links/Rechts)
2. Wähle Bein (Front/Back/III)
3. Wähle Servo (Rotation/Height)
4. Bewege Slider

**Vorteil**: Intuitive Zuordnung zum physischen Roboter!

## Migration

Keine Änderungen am Code notwendig - vollständig rückwärtskompatibel!

## Nächste Schritte (Optional)

- [ ] Preset-Positionen pro Bein speichern
- [ ] Walk-Cycle-Test-Modus
- [ ] 3D-Visualisierung der Bein-Positionen
- [ ] Individual Min/Max pro Servo

---

## Update v2.1 - Kompaktes Layout

**Änderungen zur Platzoptimierung**:

### Fenstergröße
- Reduziert von 900x750 auf **850x600 px**
- ~20% weniger Höhe

### Schriftgrößen
- Titel: 18 → **14 pt**
- Mode Indicator: 12 → **9 pt**
- Info Label: 10 → **8 pt**
- Spalten-Titel: 14 → **11 pt**
- Bein-Namen: 12 → **9 pt**
- Servo-Labels: 10/11 → **8/9 pt**
- Min/Max Labels: 8 → **7 pt**
- Buttons: 11 → **9 pt**

### Abstände (Padding)
- Header pady: 10 → **3 px**
- Mode pady: 5 → **2 px**
- Info pady: 5 → **2 px**
- Main frame: 10,10 → **5,3 px**
- Column padx: 5 → **3 px**
- Leg groups: 5,10 → **3,3 px**
- Servo controls: 5,3/5 → **3,1 px**
- Buttons: 10,10 → **5,3 px**

### Slider
- Canvas height: 500 → **400 px**
- Slider length compact: 300 → **250 px**
- Slider length normal: 500 → **400 px**
- Slider width: Standard → **8 px** (schmaler)

### Button-Texte (gekürzt)
- "Reset All to Default" → **"Reset All"**
- "All to Min" → **"All Min"**
- "All to Max" → **"All Max"**

**Ergebnis**: Deutlich kompakteres Layout bei gleicher Funktionalität!

---

**Version**: 2.1 (kompakt)
**Datum**: 2026-01-18
**Status**: ✅ Produktionsbereit
