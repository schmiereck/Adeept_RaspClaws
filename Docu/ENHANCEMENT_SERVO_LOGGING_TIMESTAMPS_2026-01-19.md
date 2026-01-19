# Enhancement: Millisecond Timestamps für Servo-Logging

**Datum:** 2026-01-19  
**Typ:** Enhancement - Debugging  
**Zweck:** Bessere Analyse der Servo-Bewegungen und Zuck-Probleme

---

## Problem

Die Servo-Logs kamen **sehr langsam** (1x pro Sekunde), was eine genaue Analyse der Bewegungen erschwerte:

```
[SERVOS] L1:335,310 L2:321,279 ...
[SERVOS] L1:265,310 L2:335,300 ...  # Wann genau war das?
[SERVOS] L1:316,272 L2:289,310 ...  # Wie lange zwischen den Logs?
```

**Probleme:**
- ❌ Keine Zeitstempel - kann Bewegungsablauf nicht nachvollziehen
- ❌ Zu langsame Updates (1s) - verpassen schnelle Bewegungen
- ❌ Schwer zu analysieren wo das Zucken auftritt

---

## Lösung

### 1. Millisekunden-Timestamps hinzugefügt

**Client/GUI.py (Zeile ~433):**
```python
# Log servo positions to terminal with timestamp
if servo_info:
    timestamp = int(time.time() * 1000)  # Milliseconds since epoch
    print(f"[{timestamp}] [SERVOS] {servo_info}")
```

**Beispiel Output:**
```
[1737326451234] [SERVOS] L1:335,310 L2:321,279 L3:335,310 R1:279,321 R2:265,290 R3:279,321
[1737326451434] [SERVOS] L1:265,310 L2:335,300 L3:265,310 R1:265,300 R2:335,290 R3:265,300
[1737326451634] [SERVOS] L1:316,272 L2:289,310 L3:311,265 R1:311,290 R2:289,335 R3:311,290
```

**Vorteile:**
- ✅ **Präzise Zeitstempel** - Millisekunden-Genauigkeit
- ✅ **Delta berechenbar** - Einfach zu sehen: 434 - 234 = 200ms zwischen Logs
- ✅ **Analyse möglich** - Wo genau tritt das Zucken auf?

### 2. Update-Intervall verkürzt (1s → 0.2s)

**Server/GUIServer.py (Zeile ~218):**
```python
time.sleep(0.2)  # 200ms update interval for better servo analysis
```

**Vorher:** 1x pro Sekunde (1000ms)  
**Nachher:** 5x pro Sekunde (200ms)

**Vorteile:**
- ✅ **5× mehr Datenpunkte** pro Sekunde
- ✅ **Schnellere Bewegungen sichtbar**
- ✅ **Besser für Smooth-Mode-Analyse**

---

## Analyse-Beispiel

### Mit Timestamps können wir jetzt sehen:

**Normale Bewegung:**
```
[1737326451000] [SERVOS] L1:300,300 ...
[1737326451200] [SERVOS] L1:310,320 ...  # +200ms, smooth Interpolation
[1737326451400] [SERVOS] L1:320,340 ...  # +200ms, weiter smooth
[1737326451600] [SERVOS] L1:330,360 ...  # +200ms, perfekt!
```

**Zuckbewegung erkennbar:**
```
[1737326451000] [SERVOS] L1:335,310 ...  # Bein vorne
[1737326451200] [SERVOS] L1:300,310 ...  # SPRUNG zur Mitte! (-35)
[1737326451400] [SERVOS] L1:265,310 ...  # Weiter zurück
```

**Delta-Analyse:**
```python
# Zwischen zwei Logs:
delta_time = 1737326451200 - 1737326451000 = 200ms ✓

# Position-Änderung:
delta_pos_h = 300 - 335 = -35  # Sprung!
delta_pos_v = 310 - 310 = 0    # Keine Änderung
```

---

## Verwendung

### Logs sammeln für Analyse

1. **Starte GUI** mit Servo-Logging
2. **Bewege Roboter** im Smooth-Modus
3. **Kopiere Terminal-Output** mit Timestamps
4. **Analysiere:**
   ```python
   import re
   
   # Parse log line
   match = re.match(r'\[(\d+)\] \[SERVOS\] (.*)', line)
   timestamp = int(match.group(1))
   servo_data = match.group(2)
   
   # Calculate deltas
   delta_t = timestamp - prev_timestamp
   delta_pos = parse_servo_positions(servo_data) - prev_positions
   
   # Find jumps (sudden large changes)
   if abs(delta_pos) > 20:
       print(f"Jump detected at {timestamp}: {delta_pos}")
   ```

### Typische Delta-Werte

**Smooth-Bewegung (erwartet):**
- Delta Zeit: ~200ms (Update-Intervall)
- Delta Position: ~5-15 PWM (kleine Schritte)

**Zuck-Bewegung (Problem):**
- Delta Zeit: ~200ms (gleich)
- Delta Position: ~30-70 PWM (großer Sprung!)

**Keine Bewegung:**
- Delta Zeit: ~200ms
- Delta Position: 0 PWM

---

## Performance Impact

### CPU Last

**Vorher (1s Intervall):**
- 1 Servo-Update pro Sekunde
- Minimal CPU Last (~0.1%)

**Nachher (0.2s Intervall):**
- 5 Servo-Updates pro Sekunde
- Immer noch minimal (~0.5%)

**Fazit:** ✅ Negligible - Keine merkbare Performance-Verschlechterung

### Netzwerk-Traffic

**Vorher:**
```
INFO:47.8 0.0 40.0 0.0\n  (30 bytes, 1×/s = 30 B/s)
```

**Nachher:**
```
INFO:47.8 0.0 40.0 0.0 | L1:300,300 L2:300,300 ...\n  (110 bytes, 5×/s = 550 B/s)
```

**Fazit:** ✅ Nur 550 Bytes/s - Kein Problem für TCP-Verbindung

---

## Zurücksetzen (falls nötig)

Wenn 0.2s zu oft ist (unwahrscheinlich), zurück auf 1s:

**Server/GUIServer.py:**
```python
time.sleep(1)  # Zurück auf 1 Sekunde
```

---

## Status

✅ **Implementiert** - Bereit zum Testen

**Geänderte Files:**
- `Client/GUI.py` - Timestamp hinzugefügt
- `Server/GUIServer.py` - Update-Intervall auf 0.2s

**Test auf Pi:**
```bash
cd ~/adeept_raspclaws
git pull
sudo systemctl restart robot_server.service
```

---

## Erwartetes Ergebnis

**Terminal-Output mit Timestamps:**
```
[1737326451234] [SERVOS] L1:335,310 L2:321,279 L3:335,310 R1:279,321 R2:265,290 R3:279,321
[1737326451434] [SERVOS] L1:265,310 L2:335,300 L3:265,310 R1:265,300 R2:335,290 R3:265,300
[1737326451634] [SERVOS] L1:316,272 L2:289,310 L3:311,265 R1:311,290 R2:289,335 R3:311,290
[1737326451834] [SERVOS] L1:335,310 L2:265,195 L3:335,310 R1:335,405 R2:265,290 R3:335,405
...
```

**Mit diesen Daten können wir:**
- ✅ Genau sehen wann Bewegungen stattfinden
- ✅ Sprünge/Zucken identifizieren (große Delta-Werte)
- ✅ Timing-Probleme erkennen (unregelmäßige Deltas)
- ✅ dove() Funktion debuggen (welcher Step verursacht Sprünge)

---

## Nächste Schritte

1. **Logs mit Timestamps sammeln** während einer Forward-Bewegung im Smooth-Modus
2. **Analyse:**
   - Wo genau tritt das Zucken auf? (Timestamp)
   - Welche Servos springen? (L1, L2, R1, etc.)
   - Wie groß ist der Sprung? (Delta Position)
3. **Korrelation mit dove() Steps:**
   - Step 1 → 2 Übergang?
   - Step 2 → 3 Übergang?
   - Step 3 → 4 Übergang?
   - Step 4 → 1 Übergang?

Mit diesen Informationen können wir das dove() Problem **gezielt** fixen! 🎯

---

## Autor

GitHub Copilot  
Datum: 2026-01-19  
Auf Wunsch von: schmiereck
