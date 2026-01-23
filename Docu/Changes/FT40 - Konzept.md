# FT40 - Konzept für vollständige Implementierung

**Datum:** 2026-01-23  
**Status:** In Planung  
**Priorität:** Hoch (aber vorsichtig!)

## Ziel

Smooth Richtungswechsel ohne Zucken - auch nach einem Stop oder während der Bewegung.

## Analyse-Ergebnis

**Aktueller Zustand:**
- ✅ Positions-Tracking vorhanden (`_leg_positions`)
- ✅ Interpolation funktioniert (alpha=0.9)
- ✅ Kontinuierliche Bewegung ist smooth
- ❌ **Richtungswechsel nach Stop führt zu Sprüngen** (Phase-Reset!)
- ❌ Richtungswechsel-Erkennung fehlt
- ❌ Variable Interpolation fehlt
- ❌ Vertikale Interpolation fehlt

## Schrittweise Implementierung

### Phase 1: Absicherung (ZUERST!)
- [x] Analyse-Dokument erstellt
- [ ] Konzept-Dokument erstellt
- [ ] Unit-Tests für aktuellen Zustand schreiben
- [ ] Manuelle Test-Checkliste erstellen
- [ ] Backup-Tag im Git erstellen (falls Git verwendet wird)

### Phase 2: Richtungswechsel-Erkennung (MINIMAL)
**Ziel:** Erkennen, wann sich Richtung oder Kommando ändert

**Änderungen:**
1. Counter für Steps seit letztem Wechsel hinzufügen
2. Richtungswechsel-Logik implementieren
3. `_last_command` und `_last_speed_sign` korrekt verwenden

**Risiko:** NIEDRIG - nur neue Variablen, keine Änderung an bestehender Logik

**Test:**
- Forward → Backward: Wechsel wird erkannt
- Forward → Stop → Forward: Wechsel wird erkannt
- Logs zeigen Richtungswechsel an

### Phase 3: Variable Alpha (MODERAT)
**Ziel:** Stärkere Interpolation bei Richtungswechsel

**Änderungen:**
1. Alpha-Berechnung basierend auf Richtungswechsel
2. Erste 5-10 Steps nach Wechsel: alpha = 0.3 → 1.0
3. Danach: alpha = 0.9 (normal)

**Risiko:** MODERAT - verändert Interpolationslogik

**Test:**
- Forward → Backward: Smooth Übergang, kein Zucken
- Backward → Forward: Smooth Übergang, kein Zucken
- Keine Regression bei kontinuierlicher Bewegung

### Phase 4: Phase-Handling bei Stop (KRITISCH!)
**Ziel:** Phase NICHT zurücksetzen oder smart zurücksetzen

**Option A (Sicher):** Phase beibehalten, aber bei langem Stop (>2s) zurücksetzen
**Option B (Riskant):** Phase nie zurücksetzen, immer von aktueller Position starten

**Änderungen:**
1. Zeile 1031: `gait_phase = 0.0` entfernen oder konditionieren
2. Evtl. Timer für "langen Stop" hinzufügen

**Risiko:** HOCH - könnte zu unerwarteten Bewegungen führen

**Test:**
- Forward → Stop (kurz) → Forward: Smooth Fortsetzung
- Forward → Stop (lang) → Backward: Smooth Start
- Stand-Position wird korrekt angefahren

### Phase 5: Vertikale Interpolation (OPTIONAL)
**Ziel:** Auch vertikale Bewegungen interpolieren

**Änderungen:**
1. `_leg_positions` erweitern: `{'h': 0, 'v': 0}` statt nur int
2. Vertikale Position ebenfalls interpolieren

**Risiko:** MODERAT - große Strukturänderung

**Test:**
- Beine heben/senken sich smooth
- Keine Sprünge in der Höhe

## Detaillierte Implementierungsschritte

### Phase 2: Richtungswechsel-Erkennung

```python
# Neue globale Variablen hinzufügen (nach Zeile 117)
_steps_since_change = 0  # Counter für Steps seit Richtungswechsel
_direction_changed = False  # Flag für Richtungswechsel
```

```python
# In move_thread() nach Zeile 989 (vor speed_left/speed_right Berechnung)
def move_thread():
    global _last_command, _last_speed_sign, _steps_since_change, _direction_changed
    
    # ... existing code ...
    
    # Detect command change
    current_command = (direction_command, turn_command)
    command_changed = (current_command != _last_command) and (_last_command is not None)
    
    # Detect direction change (forward/backward flip)
    if movement_active:
        # Calculate average direction
        current_direction_value = (speed_left + speed_right) / 2
        current_sign = 1 if current_direction_value < 0 else (-1 if current_direction_value > 0 else 0)
        
        direction_changed = (current_sign != 0 and 
                           _last_speed_sign != 0 and 
                           current_sign != _last_speed_sign)
        
        # Update tracking variables
        if command_changed or direction_changed:
            _steps_since_change = 0
            _direction_changed = True
            print(f"[MOVEMENT] Direction change detected: cmd={current_command}, sign={current_sign}")
        else:
            _steps_since_change += 1
            if _steps_since_change > 10:
                _direction_changed = False
        
        _last_command = current_command
        _last_speed_sign = current_sign
```

**Tests für Phase 2:**
```python
# test_direction_change_detection.py
def test_forward_to_backward():
    # Setup: Forward movement
    # Action: Change to Backward
    # Assert: _steps_since_change == 0, _direction_changed == True
    
def test_continuous_forward():
    # Setup: Forward movement
    # Action: Continue Forward for 20 steps
    # Assert: _steps_since_change > 10, _direction_changed == False
```

### Phase 3: Variable Alpha

```python
# In move_thread(), Zeile 1046 ersetzen
# Alte Version:
# alpha = 0.9

# Neue Version:
if _direction_changed and _steps_since_change < 5:
    # Gradual transition: 0.3 → 1.0 over 5 steps
    alpha = 0.3 + (_steps_since_change / 5.0) * 0.7
    print(f"[INTERPOLATION] Smooth transition: step={_steps_since_change}, alpha={alpha:.2f}")
else:
    alpha = 0.9  # Normal continuous movement
```

**Tests für Phase 3:**
```python
def test_alpha_during_transition():
    # Step 0: alpha should be 0.3
    # Step 1: alpha should be 0.44
    # Step 5: alpha should be 1.0
    # Step 10: alpha should be 0.9 (normal)
```

### Phase 4: Phase-Handling (Vorsichtig!)

**Option A (Empfohlen für Start):**
```python
# Zeile 1029-1032 ändern
if not movement_active:
    handle_stand_or_steady()
    # Option A: Phase nur bei langem Stop zurücksetzen
    # TODO: Implementiere Timer-basiertes Reset
    # gait_phase = 0.0  # Erstmal auskommentieren!
    return
```

**Tests für Phase 4:**
```python
def test_stop_and_restart():
    # Forward → Stop (0.1s) → Forward
    # Assert: Kein Sprung, smooth Fortsetzung
    
def test_direction_change_after_stop():
    # Forward → Stop (0.5s) → Backward
    # Assert: Smooth Übergang, kein Zucken
```

## Risiko-Matrix

| Phase | Risiko | Impact | Rollback-Schwierigkeit |
|-------|--------|--------|------------------------|
| 1 (Tests) | Niedrig | Niedrig | Einfach |
| 2 (Erkennung) | Niedrig | Niedrig | Einfach |
| 3 (Alpha) | Mittel | Mittel | Mittel |
| 4 (Phase) | **Hoch** | **Hoch** | **Schwer** |
| 5 (Vertikal) | Mittel | Mittel | Mittel |

## Test-Checkliste (Manuell)

Nach jeder Phase durchführen:

- [ ] **Forward kontinuierlich:** Smooth, kein Zucken
- [ ] **Backward kontinuierlich:** Smooth, kein Zucken
- [ ] **Forward → Stop → Forward:** Smooth Fortsetzung
- [ ] **Forward → Backward (ohne Stop):** Smooth Übergang
- [ ] **Backward → Forward (ohne Stop):** Smooth Übergang
- [ ] **Forward → Left:** Smooth Übergang
- [ ] **Left → Right:** Smooth Übergang
- [ ] **Multiple schnelle Richtungswechsel:** Keine Crashes
- [ ] **Stand-Position:** Wird korrekt angefahren
- [ ] **Steady-Mode:** Funktioniert noch

## Rollback-Plan

Falls etwas schief geht:

1. **Git:** `git checkout <backup-tag>`
2. **Kein Git:** Manuelle Backups der Dateien:
   - `Move.py` → `Move.py.backup_phase_N`
   - Tests entfernen

## Erfolgs-Kriterien

**Phase 2:** Logs zeigen Richtungswechsel korrekt an  
**Phase 3:** Forward → Backward ist smoother als vorher  
**Phase 4:** Forward → Stop → Backward hat kein Zucken mehr  
**Phase 5:** Vertikale Bewegungen sind smooth  

## Zeitplan

- **Phase 1 (Tests):** ~1-2 Stunden
- **Phase 2 (Erkennung):** ~30 Minuten
- **Phase 3 (Alpha):** ~30 Minuten
- **Phase 4 (Phase):** ~1 Stunde (viele Tests!)
- **Phase 5 (Vertikal):** ~2 Stunden (optional)

**Total:** ~5-6 Stunden (mit ausgiebigen Tests)

## Nächste Schritte

1. ✅ Dieses Konzept-Dokument erstellen
2. [ ] Unit-Tests für aktuellen Zustand schreiben
3. [ ] Manuelle Test-Checkliste durchführen (Baseline)
4. [ ] Phase 2 implementieren
5. [ ] Tests durchführen
6. [ ] Falls OK: Phase 3
7. [ ] Tests durchführen
8. [ ] Falls OK: Phase 4 (mit extra Vorsicht!)
9. [ ] Tests durchführen
10. [ ] Optional: Phase 5

## Notizen

- **Vorsicht:** Nicht mehrere Phasen gleichzeitig implementieren!
- **Tests:** Nach jeder Phase die komplette Test-Checkliste durchführen
- **Logs:** Ausgiebiges Logging für Debugging
- **Rollback:** Bei Problemen sofort zur letzten funktionierenden Version zurück

## Offene Fragen

1. Wird Git verwendet? → Backup-Strategie anpassen
2. Gibt es bereits Test-Hardware/Simulator? → Test-Strategie anpassen
3. Wie lange dauert ein typischer Stop? → Für Phase 4 Timer-Wert bestimmen
4. Ist Phase 5 (vertikale Interpolation) gewünscht? → Priorität klären
