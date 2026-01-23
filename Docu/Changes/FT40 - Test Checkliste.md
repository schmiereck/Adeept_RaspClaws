# FT40 - Manuelle Test-Checkliste

**Datum:** 2026-01-23  
**Zweck:** Manuelle Tests zur Qualitätssicherung während FT40 Implementation

## Baseline Tests (Vor Änderungen)

Diese Tests dokumentieren den **aktuellen Zustand** vor den FT40-Verbesserungen.

### Kontinuierliche Bewegung

- [ ] **Forward kontinuierlich (10 Sekunden)**
  - Erwartung: Smooth Bewegung, kein Zucken
  - Tatsächlich: ___________________
  - Status: ⚪ Nicht getestet / ✅ OK / ❌ Problem

- [ ] **Backward kontinuierlich (10 Sekunden)**
  - Erwartung: Smooth Bewegung, kein Zucken
  - Tatsächlich: ___________________
  - Status: ⚪ / ✅ / ❌

- [ ] **Left Turn kontinuierlich (10 Sekunden)**
  - Erwartung: Smooth Drehung, kein Zucken
  - Tatsächlich: ___________________
  - Status: ⚪ / ✅ / ❌

- [ ] **Right Turn kontinuierlich (10 Sekunden)**
  - Erwartung: Smooth Drehung, kein Zucken
  - Tatsächlich: ___________________
  - Status: ⚪ / ✅ / ❌

### Stop und Neustart

- [ ] **Forward → Stop (2s) → Forward**
  - Erwartung: Smooth Fortsetzung, evtl. kleiner Sprung beim Restart
  - Tatsächlich: ___________________
  - Zucken beim Stop: Ja / Nein
  - Zucken beim Restart: Ja / Nein
  - Status: ⚪ / ✅ / ❌

- [ ] **Backward → Stop (2s) → Backward**
  - Erwartung: Smooth Fortsetzung, evtl. kleiner Sprung beim Restart
  - Tatsächlich: ___________________
  - Status: ⚪ / ✅ / ❌

### Richtungswechsel OHNE Stop

- [ ] **Forward → Backward (direkt, ohne Stop)**
  - Erwartung: ❓ **BASELINE - Könnte Zucken haben**
  - Tatsächlich: ___________________
  - Zucken sichtbar: Ja / Nein
  - Intensität (1-5): ___
  - Status: ⚪ / ✅ / ❌

- [ ] **Backward → Forward (direkt, ohne Stop)**
  - Erwartung: ❓ **BASELINE - Könnte Zucken haben**
  - Tatsächlich: ___________________
  - Zucken sichtbar: Ja / Nein
  - Intensität (1-5): ___
  - Status: ⚪ / ✅ / ❌

### Richtungswechsel MIT Stop

- [ ] **Forward → Stop (1s) → Backward**
  - Erwartung: ❓ **BASELINE - Könnte Zucken haben**
  - Tatsächlich: ___________________
  - Zucken beim Stop: Ja / Nein
  - Zucken beim Restart: Ja / Nein
  - Intensität (1-5): ___
  - Status: ⚪ / ✅ / ❌

- [ ] **Backward → Stop (1s) → Forward**
  - Erwartung: ❓ **BASELINE - Könnte Zucken haben**
  - Tatsächlich: ___________________
  - Status: ⚪ / ✅ / ❌

### Bewegungsübergänge

- [ ] **Forward → Left Turn**
  - Erwartung: Smooth Übergang
  - Tatsächlich: ___________________
  - Status: ⚪ / ✅ / ❌

- [ ] **Forward → Right Turn**
  - Erwartung: Smooth Übergang
  - Tatsächlich: ___________________
  - Status: ⚪ / ✅ / ❌

- [ ] **Left Turn → Right Turn**
  - Erwartung: Smooth Übergang
  - Tatsächlich: ___________________
  - Status: ⚪ / ✅ / ❌

### Spezial-Szenarien

- [ ] **Schnelle Richtungswechsel (Forward → Backward → Forward → Backward in 2s)**
  - Erwartung: ❓ System sollte stabil bleiben, könnte aber zucken
  - Tatsächlich: ___________________
  - Crashes: Ja / Nein
  - Zucken: Ja / Nein
  - Status: ⚪ / ✅ / ❌

- [ ] **Stand-Position nach Bewegung**
  - Erwartung: Beine fahren smooth in Stand-Position
  - Tatsächlich: ___________________
  - Status: ⚪ / ✅ / ❌

- [ ] **Steady-Mode nach Bewegung**
  - Erwartung: Steady-Mode funktioniert normal
  - Tatsächlich: ___________________
  - Status: ⚪ / ✅ / ❌

---

## Nach Phase 2: Richtungswechsel-Erkennung

Diese Tests prüfen, ob die Erkennung funktioniert (noch keine Verhaltensänderung erwartet).

- [ ] **Logs zeigen Richtungswechsel an**
  - Forward → Backward: Log-Meldung erscheint
  - Backward → Forward: Log-Meldung erscheint
  - Forward → Left: Log-Meldung erscheint

- [ ] **Keine Regression bei kontinuierlicher Bewegung**
  - Alle Baseline-Tests von oben wiederholen
  - Status: ⚪ / ✅ / ❌

---

## Nach Phase 3: Variable Alpha

Diese Tests prüfen, ob die variable Interpolation Verbesserungen bringt.

### Richtungswechsel - Erwartung: VERBESSERT

- [ ] **Forward → Backward (ohne Stop)**
  - Erwartung: ✅ **BESSER als Baseline - Weniger Zucken**
  - Tatsächlich: ___________________
  - Zucken sichtbar: Ja / Nein
  - Intensität (1-5): ___ (Vergleich zu Baseline: ___)
  - Verbesserung: Ja / Nein / Unklar
  - Status: ⚪ / ✅ / ❌

- [ ] **Backward → Forward (ohne Stop)**
  - Erwartung: ✅ **BESSER als Baseline**
  - Tatsächlich: ___________________
  - Verbesserung vs Baseline: Ja / Nein / Unklar
  - Status: ⚪ / ✅ / ❌

- [ ] **Forward → Stop → Backward**
  - Erwartung: ✅ **BESSER als Baseline** (bei Restart)
  - Tatsächlich: ___________________
  - Verbesserung vs Baseline: Ja / Nein / Unklar
  - Status: ⚪ / ✅ / ❌

### Regression Tests

- [ ] **Forward kontinuierlich**
  - Erwartung: ✅ Gleich gut wie Baseline
  - Tatsächlich: ___________________
  - Regression: Ja / Nein
  - Status: ⚪ / ✅ / ❌

- [ ] **Backward kontinuierlich**
  - Erwartung: ✅ Gleich gut wie Baseline
  - Tatsächlich: ___________________
  - Regression: Ja / Nein
  - Status: ⚪ / ✅ / ❌

- [ ] **Forward → Stop → Forward**
  - Erwartung: ✅ Gleich gut wie Baseline
  - Tatsächlich: ___________________
  - Regression: Ja / Nein
  - Status: ⚪ / ✅ / ❌

---

## Nach Phase 4: Phase-Handling (KRITISCH!)

Diese Tests sind **extra wichtig**, da Phase 4 hohes Risiko hat.

### Richtungswechsel nach Stop - Erwartung: DEUTLICH VERBESSERT

- [ ] **Forward → Stop (kurz, 0.5s) → Backward**
  - Erwartung: ✅ **DEUTLICH BESSER - Kein Zucken mehr!**
  - Tatsächlich: ___________________
  - Zucken beim Restart: Ja / Nein
  - Intensität (1-5): ___ (Baseline war: ___)
  - **KRITISCH:** Massive Verbesserung erwartet!
  - Status: ⚪ / ✅ / ❌

- [ ] **Backward → Stop (kurz, 0.5s) → Forward**
  - Erwartung: ✅ **DEUTLICH BESSER - Kein Zucken mehr!**
  - Tatsächlich: ___________________
  - Status: ⚪ / ✅ / ❌

- [ ] **Forward → Stop (kurz, 0.5s) → Forward**
  - Erwartung: ✅ Smooth Fortsetzung (sollte schon vorher OK gewesen sein)
  - Tatsächlich: ___________________
  - Status: ⚪ / ✅ / ❌

### Langer Stop (falls Timer implementiert)

- [ ] **Forward → Stop (lang, 3s) → Forward**
  - Erwartung: ✅ Smooth Start (evtl. kleiner Sprung OK)
  - Tatsächlich: ___________________
  - Status: ⚪ / ✅ / ❌

- [ ] **Forward → Stop (lang, 3s) → Backward**
  - Erwartung: ✅ Smooth Start
  - Tatsächlich: ___________________
  - Status: ⚪ / ✅ / ❌

### Stand-Position (KRITISCH!)

- [ ] **Forward → Stop → Stand-Position wird angefahren**
  - Erwartung: ✅ Beine fahren korrekt in Stand-Position
  - Tatsächlich: ___________________
  - **KRITISCH:** Darf nicht kaputt sein!
  - Beine in korrekter Position: Ja / Nein
  - Status: ⚪ / ✅ / ❌

- [ ] **Backward → Stop → Stand-Position**
  - Erwartung: ✅ Beine fahren korrekt in Stand-Position
  - Tatsächlich: ___________________
  - Status: ⚪ / ✅ / ❌

### Steady-Mode (KRITISCH!)

- [ ] **Steady-Mode aktivieren nach Bewegung**
  - Erwartung: ✅ Funktioniert normal
  - Tatsächlich: ___________________
  - **KRITISCH:** Darf nicht kaputt sein!
  - Status: ⚪ / ✅ / ❌

### Regression Tests (WICHTIG!)

- [ ] **Forward kontinuierlich**
  - Erwartung: ✅ Gleich gut wie vorher
  - Tatsächlich: ___________________
  - ❌ Regression: Ja / Nein
  - Status: ⚪ / ✅ / ❌

- [ ] **Backward kontinuierlich**
  - Erwartung: ✅ Gleich gut wie vorher
  - Tatsächlich: ___________________
  - ❌ Regression: Ja / Nein
  - Status: ⚪ / ✅ / ❌

- [ ] **Left Turn kontinuierlich**
  - Erwartung: ✅ Gleich gut wie vorher
  - Tatsächlich: ___________________
  - ❌ Regression: Ja / Nein
  - Status: ⚪ / ✅ / ❌

- [ ] **Right Turn kontinuierlich**
  - Erwartung: ✅ Gleich gut wie vorher
  - Tatsächlich: ___________________
  - ❌ Regression: Ja / Nein
  - Status: ⚪ / ✅ / ❌

---

## Nach Phase 5: Vertikale Interpolation (Optional)

Diese Tests prüfen vertikale Bewegungen.

- [ ] **Beine heben sich smooth**
  - Erwartung: ✅ Keine Sprünge in der Höhe
  - Tatsächlich: ___________________
  - Status: ⚪ / ✅ / ❌

- [ ] **Beine senken sich smooth**
  - Erwartung: ✅ Keine Sprünge in der Höhe
  - Tatsächlich: ___________________
  - Status: ⚪ / ✅ / ❌

- [ ] **Forward kontinuierlich (vertikale Bewegung beachten)**
  - Erwartung: ✅ Smooth Heben/Senken während Gang
  - Tatsächlich: ___________________
  - Status: ⚪ / ✅ / ❌

---

## Rollback-Kriterien

**Falls eines dieser Probleme auftritt, SOFORT ROLLBACK:**

- ❌ **Crash oder Freeze** während normaler Bewegung
- ❌ **Stand-Position funktioniert nicht mehr** korrekt
- ❌ **Steady-Mode funktioniert nicht mehr**
- ❌ **Deutliche Regression** bei kontinuierlicher Bewegung (Forward/Backward)
- ❌ **Servos fahren in extreme Positionen** (Gefahr für Hardware!)

## Erfolgs-Kriterien

**Phase 2:** Logs zeigen Richtungswechsel, keine Regression  
**Phase 3:** Richtungswechsel sind spürbar smoother  
**Phase 4:** Richtungswechsel nach Stop sind deutlich besser, keine Regression  
**Phase 5:** Vertikale Bewegungen sind smooth  

---

## Test-Protokoll

### Baseline Tests
- Datum: ___________
- Tester: ___________
- Ergebnis: ⚪ / ✅ / ❌
- Notizen: _____________________________

### Phase 2 Tests
- Datum: ___________
- Tester: ___________
- Ergebnis: ⚪ / ✅ / ❌
- Notizen: _____________________________

### Phase 3 Tests
- Datum: ___________
- Tester: ___________
- Ergebnis: ⚪ / ✅ / ❌
- Notizen: _____________________________

### Phase 4 Tests
- Datum: ___________
- Tester: ___________
- Ergebnis: ⚪ / ✅ / ❌
- Notizen: _____________________________

### Phase 5 Tests (optional)
- Datum: ___________
- Tester: ___________
- Ergebnis: ⚪ / ✅ / ❌
- Notizen: _____________________________
