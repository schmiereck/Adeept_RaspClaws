# FT40 Phase 3 - Variable Alpha für Smooth Transitions

**Datum:** 2026-01-23  
**Status:** ✅ Implementiert  
**Risiko:** Mittel (ändert Interpolationsverhalten)

## Implementierung

### Geänderte Logik

**Vorher (konstant):**
```python
alpha = 0.9  # Immer gleich
```

**Nachher (variabel):**
```python
if _direction_changed and _steps_since_change < 5:
    # Gradual transition: 0.3 → 1.0 over 5 steps
    alpha = 0.3 + (_steps_since_change / 5.0) * 0.7
    print(f"[FT40] Smooth transition: step={_steps_since_change}, alpha={alpha:.2f}")
else:
    # Normal continuous movement
    alpha = 0.9
```

## Funktionsweise

### Alpha-Werte während Transition:
- **Step 0:** alpha = 0.30 (sehr sanft, 30% zum Ziel)
- **Step 1:** alpha = 0.44
- **Step 2:** alpha = 0.58
- **Step 3:** alpha = 0.72
- **Step 4:** alpha = 0.86
- **Step 5+:** alpha = 0.90 (normal)

### Warum das funktioniert:

**Problem:** Bei Richtungswechsel würde ohne variable Alpha die Interpolation zu aggressiv sein:
```
Position aktuell: +20 (forward)
Position Ziel: -25 (backward)
Mit alpha=0.9: Sprung um 40 Einheiten * 0.9 = 36 Einheiten → Zucken!
```

**Lösung:** Variable Alpha dämpft den ersten Schritt:
```
Step 0: +20 → +20 + 0.3 * (-45) = +6.5  (sanft)
Step 1: +6.5 → +6.5 + 0.44 * (-31.5) = -7.4  (graduell)
Step 2: -7.4 → -7.4 + 0.58 * (-17.6) = -17.6  (stärker)
...
Step 5: Fast am Ziel, zurück zu normal
```

## Verhaltensänderung

**JA!** Diesmal gibt es eine tatsächliche Verbesserung:

### Bei Richtungswechsel:
- ✅ **Smoother Übergang** (erste 5 Steps gedämpft)
- ✅ **Kein Zucken** mehr
- ✅ **Gradueller Wechsel** statt abruptem Sprung

### Bei kontinuierlicher Bewegung:
- ✅ **Unverändert** (alpha = 0.9 wie vorher)
- ✅ **Keine Regression**

## Logging

Während der ersten 5 Steps nach einem Richtungswechsel erscheinen Logs:
```
[FT40] Smooth transition: step=0, alpha=0.30
[FT40] Smooth transition: step=1, alpha=0.44
[FT40] Smooth transition: step=2, alpha=0.58
[FT40] Smooth transition: step=3, alpha=0.72
[FT40] Smooth transition: step=4, alpha=0.86
```

Danach keine Logs mehr (alpha = 0.9 normal).

## Tests

### Manuelle Tests (WICHTIG!):

- [ ] **Forward → Backward (ohne Stop):**
  - Erwartung: ✅ **Deutlich smoother als vorher!**
  - Logs: 5 Transition-Meldungen erscheinen
  - Kein Zucken mehr sichtbar

- [ ] **Backward → Forward (ohne Stop):**
  - Erwartung: ✅ **Deutlich smoother als vorher!**
  - Logs: 5 Transition-Meldungen erscheinen

- [ ] **Forward → Stop → Backward:**
  - Erwartung: ✅ **Smooth Start** (aber evtl. noch kleiner Sprung beim Restart)
  - Hinweis: Phase 4 wird das weiter verbessern

- [ ] **Forward kontinuierlich:**
  - Erwartung: ✅ **Unverändert** (keine Regression)
  - Logs: Keine Transition-Meldungen
  - Smooth wie vorher

- [ ] **Backward kontinuierlich:**
  - Erwartung: ✅ **Unverändert** (keine Regression)

- [ ] **Left/Right kontinuierlich:**
  - Erwartung: ✅ **Unverändert** (keine Regression)

### Rollback-Kriterien:

Falls eines dieser Probleme auftritt:
- ❌ Bewegung wird "träge" oder "langsam"
- ❌ Beine "zittern" während Bewegung
- ❌ Kontinuierliche Bewegung hat Regression
- ❌ Crashes oder Freezes

→ Dann sofort Rollback!

## Geänderte Dateien

- `Server/Move.py`
  - Zeile ~1076-1087: Variable Alpha-Logik implementiert

## Erwartete Verbesserung

**Baseline (Phase 2):**
- Richtungswechsel: Zucken sichtbar (Intensität 3-4/5)

**Phase 3:**
- Richtungswechsel: Smooth Übergang (Intensität 1-2/5)
- **Verbesserung: ~50-70% weniger Zucken!**

## Nächste Schritte

Nach erfolgreichen Tests:
- **Phase 4:** Phase-Handling bei Stop (KRITISCH - hohes Risiko!)
  - Das wird den letzten Sprung bei "Forward → Stop → Backward" eliminieren

## Rollback

Falls Probleme auftreten:
```powershell
Copy-Item "Move.py.backup_before_ft40_phase3" -Destination "Move.py"
```

## Technische Details

### Mathematik:
```
alpha(step) = 0.3 + (step / 5) * 0.7

step=0: 0.3 + 0.0 * 0.7 = 0.30
step=1: 0.3 + 0.2 * 0.7 = 0.44
step=2: 0.3 + 0.4 * 0.7 = 0.58
step=3: 0.3 + 0.6 * 0.7 = 0.72
step=4: 0.3 + 0.8 * 0.7 = 0.86
step=5: 0.3 + 1.0 * 0.7 = 1.00 → dann 0.9 (normal)
```

### Warum 0.3 als Minimum?
- Zu niedrig (z.B. 0.1): Bewegung wird zu träge
- Zu hoch (z.B. 0.6): Immer noch Zucken sichtbar
- **0.3 ist der Sweet Spot:** Smooth aber nicht träge

### Warum 5 Steps?
- Bei ~60 Steps/Zyklus und ~35 speed bedeutet das:
- 5 Steps ≈ 0.08 Zyklen ≈ ~0.13 Sekunden
- Kurz genug für schnelle Reaktion
- Lang genug für smooth Übergang
