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

### 5. ✅ Vergrößertes Fenster

- **Vorher**: 600x700 px
- **Nachher**: 900x750 px
- Bessere Nutzung des horizontalen Platzes

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

**Version**: 2.0
**Datum**: 2026-01-18
**Status**: ✅ Produktionsbereit
