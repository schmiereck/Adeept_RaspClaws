# FT40 Phase 4.1 - Verbesserung: Sanfteres Alpha für Richtungswechsel

**Datum:** 2026-01-23  
**Typ:** Bugfix / Minor Enhancement  
**Priorität:** Niedrig (Feintuning)

## Problem

Nach Hardware-Tests von Phase 4 wurde festgestellt:
- ✅ Alle Bewegungen funktionieren wunderbar
- ⚠️ **Beim Wechsel zwischen Forward/Backward gibt es manchmal eine kurze Rückwärts-Bewegung des Roboter-Körpers**

### Root Cause

Das Problem liegt in der Natur des Richtungswechsels:

**Forward → Backward Wechsel:**
```python
# Forward
speed_left = -35  → h_left_air = int(-35 * cos(t*π))
# Backward
speed_left = +35  → h_left_air = int(+35 * cos(t*π))
```

**Was passiert:**
1. Bei Forward: Beine bewegen sich mit negativem Vorzeichen
2. Bei Backward: Vorzeichen ändert sich sofort auf positiv
3. Die **Ziel-Position** kehrt sich sofort um
4. Obwohl `_leg_positions` die alten Werte haben, führt die Interpolation mit alpha=0.3 zu einem **sichtbaren "Ruck"** in die entgegengesetzte Richtung

**Beispiel:**
```
Forward bei Phase 0.7:
- Bein L1 aktuell bei Position: +20
- Ziel (Forward): +25

→ Wechsel zu Backward

Backward bei Phase 0.7:
- Bein L1 aktuell bei Position: +20 (aus _leg_positions)
- Ziel (Backward): -25

Mit alpha=0.3:
- Neue Position: +20 + 0.3 * (-25 - 20) = +20 + 0.3 * (-45) = +6.5
- **Bewegung: +20 → +6.5 = -13.5 (kurzes "Zurückzucken")**
```

## Lösung

**Minimale Änderung:** Alpha im ersten Step weiter reduzieren

### Vorher (Phase 3):
```python
alpha = 0.3 + (steps / 5.0) * 0.7
# Step 0: alpha = 0.30
# Step 1: alpha = 0.44
# ...
```

### Nachher (Phase 4.1):
```python
alpha = 0.2 + (steps / 5.0) * 0.8
# Step 0: alpha = 0.20  (← sanfter!)
# Step 1: alpha = 0.36
# Step 2: alpha = 0.52
# Step 3: alpha = 0.68
# Step 4: alpha = 0.84
# Step 5: alpha = 1.00 → 0.9
```

**Effekt:**
```
Mit alpha=0.2:
- Neue Position: +20 + 0.2 * (-45) = +20 - 9 = +11
- **Bewegung: +20 → +11 = -9 (sanfter!)**

Statt -13.5 nur noch -9 → ~33% weniger "Ruck"
```

## Implementierung

**Geänderte Zeile in Move.py (~Zeile 1084):**
```python
# VORHER:
alpha = 0.3 + (_steps_since_change / 5.0) * 0.7

# NACHHER:
alpha = 0.2 + (_steps_since_change / 5.0) * 0.8
```

**Kommentar erweitert:**
```python
# Extra gentle first step (0.2) reduces "backward jerk" when switching forward/backward
```

## Tests angepasst

**test_ft40.py - test_variable_alpha_calculation:**
```python
# Alpha-Werte aktualisiert:
(0, True, 0.20),  # statt 0.30
(1, True, 0.36),  # statt 0.44
(2, True, 0.52),  # statt 0.58
(3, True, 0.68),  # statt 0.72
(4, True, 0.84),  # statt 0.86
```

## Erwartete Verbesserung

**Phase 4 (vorher):**
- Kurzes Zurückzucken sichtbar (Intensität: 1-2/5)

**Phase 4.1 (nachher):**
- Noch sanfter, kaum noch sichtbar (Intensität: 0-1/5)
- **~33% weniger "Ruck"** im ersten Step

## Trade-offs

### ✅ Vorteile:
- Sanfterer Richtungswechsel
- Kaum noch Zurückzucken sichtbar

### ⚠️ Nachteile:
- Minimal langsamere Reaktion (nur 1-2 Steps Unterschied)
- Praktisch vernachlässigbar

### Warum nicht noch niedriger (z.B. alpha=0.1)?
- Zu niedrig würde Bewegung "träge" machen
- 0.2 ist der Sweet Spot: Sanft aber nicht träge
- Bei schnellen Bewegungen würde 0.1 zu spürbar verzögert reagieren

## Testing

### Hardware-Test (vom User durchgeführt):
- ✅ Alle Bewegungen funktionieren wunderbar
- ⚠️ Manchmal kurze Rückwärts-Bewegung bei Forward/Backward-Wechsel

### Erwartetes Verhalten nach Fix:
- ✅ Rückwärts-Bewegung sollte deutlich weniger/kaum noch sichtbar sein
- ✅ Keine Regression bei anderen Bewegungen

## Geänderte Dateien

- `Server/Move.py`
  - Zeile ~1084: Alpha von 0.3 auf 0.2 reduziert
  - Zeile ~1084: Formel von `0.3 + x * 0.7` auf `0.2 + x * 0.8` angepasst
  - Kommentar erweitert

- `Server/test_ft40.py`
  - Test `test_variable_alpha_calculation`: Alpha-Werte aktualisiert

## Verweise

- **Basis:** FT40 Phase 3 (Variable Alpha)
- **Build on:** FT40 Phase 4 (Smart Phase-Reset)
- **Type:** Feintuning / Bugfix

## Status

✅ Implementiert  
⏳ Hardware-Test ausstehend  

## Rollback

Falls die Änderung Probleme verursacht (z.B. Bewegung wird zu träge):

**Einfach zurück zu alpha=0.3:**
```python
alpha = 0.3 + (_steps_since_change / 5.0) * 0.7
```

Backup verfügbar: `Move.py.backup_before_ft40_phase4`

## Lessons Learned

1. **Hardware-Tests sind essentiell:** Unit-Tests können Logik validieren, aber nicht das tatsächliche Bewegungsverhalten
2. **Feintuning ist ein iterativer Prozess:** Manchmal braucht es mehrere Anläufe, um den perfekten Wert zu finden
3. **Kleine Änderungen, große Wirkung:** Von 0.3 auf 0.2 (nur 0.1 Unterschied) kann ~33% Verbesserung bringen
4. **Balance ist wichtig:** Zu niedrig = träge, zu hoch = ruckartig → Sweet Spot finden

## Zusammenfassung

**Änderung:** Alpha im ersten Step von 0.3 auf 0.2 reduziert  
**Grund:** Reduziert "Rückwärts-Ruck" bei Forward/Backward-Wechsel  
**Erwartung:** ~33% weniger sichtbares Zucken  
**Risiko:** Sehr niedrig (minimal langsamere Reaktion)  
**Status:** Bereit für Hardware-Test
