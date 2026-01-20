# FT49 - Feature: Variable Speed Control mit GUI-Slider

**Datum:** 2026-01-20
**Typ:** Feature
**Priorität:** Mittel
**Komponente:** Client/GUI.py, Server/GUIServer.py, Server/Move.py, protocol.py

## Anforderung

Benutzer sollen die Bewegungsgeschwindigkeit des Roboters während der Laufzeit dynamisch über einen Schieberegler in der GUI anpassen können, ohne den Code zu ändern oder die Anwendung neu zu starten.

## Motivation

Bisher waren die Bewegungsgeschwindigkeiten hardcodiert:
- Forward/Backward: `speed = 35`
- Turn Left/Right: `speed = 40`

**Probleme:**
- Keine Anpassung an unterschiedliche Untergründe (glatt vs. rau)
- Keine Anpassung an verschiedene Batteriezustände (volle vs. schwache Batterie)
- Kein Testen unterschiedlicher Geschwindigkeiten ohne Code-Änderungen
- Keine Möglichkeit für präzise, langsame Bewegungen bei Bedarf

## Lösung

Implementierung eines dynamischen Speed-Control-Systems mit drei Komponenten:
1. **GUI-Slider** für Benutzereingabe
2. **Echtzeit-Übertragung** zum Server
3. **Dynamische Anwendung** in Bewegungsfunktionen

### Architektur-Diagramm

```
┌─────────────────────────────────────────────────────────────────┐
│                         GUI Client                              │
│                                                                 │
│  ┌──────────┐   ┌──────────────────┐   ┌──────────────┐      │
│  │  Speed   │──▶│  on_speed_change │──▶│ send_command │      │
│  │  Slider  │   │    (callback)    │   │              │      │
│  │ (10-60)  │   └──────────────────┘   └──────┬───────┘      │
│  └──────────┘                                  │              │
└────────────────────────────────────────────────┼──────────────┘
                                                  │
                                        "setSpeed:35" (TCP)
                                                  │
┌────────────────────────────────────────────────▼──────────────┐
│                      GUIServer                                 │
│                                                                │
│  ┌──────────────────────┐      ┌────────────────────────┐    │
│  │ process_client_cmd   │─────▶│  handle_speed_command  │    │
│  │                      │      │  • Parse "setSpeed:XX" │    │
│  └──────────────────────┘      │  • Validate (10-60)    │    │
│                                 │  • Call Move module    │    │
│                                 └───────────┬────────────┘    │
└─────────────────────────────────────────────┼─────────────────┘
                                              │
                                  move.set_movement_speed(35)
                                              │
┌─────────────────────────────────────────────▼─────────────────┐
│                        Move Module                             │
│                                                                │
│  ┌───────────────────────┐      ┌──────────────────────┐     │
│  │ set_movement_speed()  │─────▶│  movement_speed = 35 │     │
│  │ • Clamp to 10-60      │      │  (global variable)   │     │
│  │ • Update global var   │      └──────────┬───────────┘     │
│  └───────────────────────┘                 │                 │
│                                            │                 │
│  ┌───────────────────────┐                 ▼                 │
│  │   move_thread()       │      Uses movement_speed for:    │
│  │                       │      • Forward: -movement_speed   │
│  │  Called every ~10ms   │      • Backward: +movement_speed  │
│  │  by RobotM thread     │      • Turn L/R: movement_speed   │
│  └───────────────────────┘                                   │
└────────────────────────────────────────────────────────────────┘
```

## Implementierung

### 1. Protocol Extension (protocol.py)

**Neues Command hinzugefügt:**

```python
# Speed commands
CMD_FAST = 'fast'              # Legacy (deprecated)
CMD_SLOW = 'slow'              # Legacy (deprecated)
CMD_SET_SPEED = 'setSpeed:'    # NEW: Variable speed control
```

**Format:** `setSpeed:35` (Wert zwischen 10 und 60)

### 2. GUI Client (Client/GUI.py)

#### A. Globale Variablen (Zeile 55-62)

```python
speed_slider_widget = None
speed_value_label_widget = None
current_speed = 35  # Default movement speed
```

#### B. Callback-Funktion (Zeile 185-198)

```python
def on_speed_change(value):
	"""Handle speed slider value change"""
	global current_speed, speed_value_label_widget

	speed = int(float(value))
	current_speed = speed

	# Update value label
	if speed_value_label_widget:
		speed_value_label_widget.config(text=str(speed))

	# Send speed command to server
	send_command(f'{CMD_SET_SPEED}{speed}')
	print(f"[Speed] Set to {speed}")
```

**Funktionsweise:**
- Wird bei jeder Slider-Bewegung aufgerufen
- Rundet Wert auf Integer
- Aktualisiert das Label neben dem Slider
- Sendet `setSpeed:XX` Command sofort zum Server
- Gibt Debug-Ausgabe in Console

#### C. GUI-Komponenten (Zeile 1149-1166)

```python
# Speed control slider
speed_label = tk.Label(root, text='Speed:', fg=color_text, bg='#212121', width=6)
speed_label.place(x=30, y=275)

speed_slider = tk.Scale(root, from_=10, to=60, orient=tk.HORIZONTAL,
                        length=200, fg=color_text, bg='#37474F',
                        troughcolor='#263238', highlightthickness=0,
                        command=lambda val: on_speed_change(val))
speed_slider.set(35)  # Default speed
speed_slider.place(x=80, y=270)

speed_value_label = tk.Label(root, text='35', fg=color_text, bg='#212121', width=3)
speed_value_label.place(x=285, y=275)

# Store references globally for updates
global speed_slider_widget, speed_value_label_widget
speed_slider_widget = speed_slider
speed_value_label_widget = speed_value_label
```

**Layout:**
```
┌─────────────────────────────────────────────┐
│  Movement Buttons (Forward/Back/Left/Right) │
│                                             │
│  Speed: ▬▬▬▬▬▬●▬▬▬▬▬▬▬▬  35                │
│         ^              ^   ^                │
│       Label         Slider Value            │
└─────────────────────────────────────────────┘
```

**Position:** Direkt unterhalb der Movement-Buttons (y=270)

**Eigenschaften:**
- **Range:** 10 (sehr langsam) bis 60 (sehr schnell)
- **Default:** 35 (ausbalanciert)
- **Farben:** Dunkles Theme-konform (#37474F / #263238)
- **Orientierung:** Horizontal
- **Breite:** 200px

### 3. Server Command Handling (Server/GUIServer.py)

#### Erweiterte handle_speed_command() Funktion (Zeile 355-378)

```python
def handle_speed_command(data):
	"""
	Handle speed control commands.

	Supports:
	- CMD_SET_SPEED: Set custom movement speed (format: "setSpeed:35")
	- CMD_FAST/CMD_SLOW: Legacy commands (deprecated, kept for backwards compatibility)
	"""
	if data.startswith(CMD_SET_SPEED):
		# Extract speed value from "setSpeed:35"
		try:
			speed_str = data[len(CMD_SET_SPEED):]
			speed = int(speed_str)
			# Clamp speed to valid range (10-60)
			speed = max(10, min(60, speed))
			print(f"[GUIServer] Setting movement speed to {speed}")
			move.set_movement_speed(speed)
			return True
		except ValueError:
			print(f"[GUIServer] Invalid speed value: {data}")
			return False
	elif data == CMD_FAST or data == CMD_SLOW:
		# Deprecated: Just acknowledge for backwards compatibility
		tcpCliSock.send(data.encode())
		return True
	return False
```

**Funktionsweise:**
1. **Parsing:** Extrahiert Wert aus `setSpeed:XX` String
2. **Validierung:** Konvertiert zu Integer, prüft ValueError
3. **Clamping:** Begrenzt auf 10-60 Range (Sicherheit)
4. **Forwarding:** Ruft `move.set_movement_speed()` auf
5. **Logging:** Debug-Ausgabe für Nachvollziehbarkeit

### 4. Movement Control (Server/Move.py)

#### A. Globale Variable (Zeile 1522)

```python
movement_speed = 35  # Default movement speed (10-60 range)
```

#### B. Setter-Funktion (Zeile 1527-1541)

```python
def set_movement_speed(speed):
	"""
	Set the movement speed for walking and turning.

	Args:
		speed: Movement speed (10-60 range)
		       - 10: Very slow, careful movements
		       - 35: Default speed (balanced)
		       - 60: Fast movements (may be less stable)
	"""
	global movement_speed
	# Clamp to valid range
	speed = max(10, min(60, int(speed)))
	movement_speed = speed
	print(f"[Move] Movement speed set to {movement_speed}")
```

**Features:**
- **Double Validation:** Auch hier Clamping (Defense in Depth)
- **Integer-Konvertierung:** Verhindert Float-Werte
- **Logging:** Server-seitige Bestätigung

#### C. Verwendung in move_thread() (Zeile 1609-1619)

**Vorher:**
```python
if direction_command == CMD_FORWARD and turn_command == MOVE_NO:
	execute_movement_step(-35, 'no')  # Hardcoded!
	movement_executed = True
elif direction_command == CMD_BACKWARD and turn_command == MOVE_NO:
	execute_movement_step(35, 'no')   # Hardcoded!
	movement_executed = True

if turn_command != MOVE_NO:
	execute_movement_step(40, turn_command)  # Hardcoded!
	movement_executed = True
```

**Nachher:**
```python
if direction_command == CMD_FORWARD and turn_command == MOVE_NO:
	execute_movement_step(-movement_speed, 'no')  # Dynamic!
	movement_executed = True
elif direction_command == CMD_BACKWARD and turn_command == MOVE_NO:
	execute_movement_step(movement_speed, 'no')   # Dynamic!
	movement_executed = True

if turn_command != MOVE_NO:
	execute_movement_step(movement_speed, turn_command)  # Dynamic!
	movement_executed = True
```

**Wichtig:** Auch in den Helper-Funktionen `handle_direction_movement()` und `handle_turn_movement()` (Zeile 1570-1591) wurde die Änderung angewendet.

## Geschwindigkeitsbereiche

| Speed | Beschreibung | Anwendungsfall |
|-------|-------------|----------------|
| **10** | Sehr langsam | Präzise Positionierung, enge Räume, schwierige Untergründe |
| **20** | Langsam | Vorsichtige Bewegungen, Balance-Tests |
| **35** | Standard (Default) | Normaler Betrieb, guter Kompromiss |
| **45** | Schnell | Freie Flächen, höhere Geschwindigkeit gewünscht |
| **60** | Sehr schnell | Maximale Geschwindigkeit (kann instabil werden!) |

**Empfehlung:**
- **Glatte Oberflächen:** 35-45 (stabiles Laufen)
- **Raue Oberflächen:** 25-35 (mehr Stabilität)
- **Schwache Batterie:** 20-30 (Servos haben weniger Kraft)
- **Volle Batterie:** 35-50 (volle Performance)

## Testing

### Testfälle

#### ✅ Test 1: Slider-Funktionalität
```
1. GUI starten
2. Slider bewegen von 35 → 50
3. Beobachten: Label zeigt "50"
4. Console-Output: "[Speed] Set to 50"
```

#### ✅ Test 2: Echtzeit-Übertragung
```
1. Forward-Button drücken (bei speed=35)
2. Während Bewegung: Slider auf 20 setzen
3. Beobachten: Bewegung wird sofort langsamer
4. Server-Console: "[GUIServer] Setting movement speed to 20"
                  "[Move] Movement speed set to 20"
```

#### ✅ Test 3: Range-Clamping
```
1. Versuche, Slider auf <10 zu setzen → bleibt bei 10
2. Versuche, Slider auf >60 zu setzen → bleibt bei 60
3. Manuelles senden von "setSpeed:100" → Server clampt auf 60
```

#### ✅ Test 4: Alle Bewegungsarten
```
1. Setze Speed auf 20
2. Teste Forward → langsam
3. Teste Backward → langsam
4. Teste Left Turn → langsam
5. Teste Right Turn → langsam
6. Setze Speed auf 50
7. Wiederhole alle Bewegungen → schnell
```

#### ✅ Test 5: Extreme Werte
```
Speed 10: Robot läuft, aber sehr langsam (stabil)
Speed 60: Robot läuft schnell (kann wackelig werden)
```

## Vorteile

### 1. Benutzerfreundlichkeit
✅ **Keine Code-Änderungen:** Geschwindigkeit direkt in GUI anpassbar
✅ **Echtzeit-Feedback:** Sofortige Anwendung bei nächster Bewegung
✅ **Visuelles Feedback:** Aktueller Wert immer sichtbar

### 2. Flexibilität
✅ **Anpassung an Untergrund:** Langsam auf Teppich, schnell auf Fliesen
✅ **Batteriezustand:** Geschwindigkeit bei schwacher Batterie reduzieren
✅ **Testing:** Verschiedene Speeds testen ohne Neustart

### 3. Stabilität
✅ **Validation auf Client:** GUI limitiert Range
✅ **Validation auf Server:** GUIServer clampt Wert
✅ **Validation in Move:** Doppelte Absicherung (Defense in Depth)

### 4. Performance
✅ **Kein Overhead:** Nur bei Slider-Änderung (nicht bei jeder Bewegung)
✅ **Asynchron:** Server verarbeitet Speed-Update ohne Movement-Thread zu blockieren

## Geänderte Dateien

### protocol.py
- **Zeile 36:** `CMD_SET_SPEED = 'setSpeed:'` hinzugefügt

### Client/GUI.py
- **Zeile 55-62:** Globale Variablen für Slider-Widgets
- **Zeile 185-198:** `on_speed_change()` Callback-Funktion
- **Zeile 1149-1166:** GUI-Komponenten (Label, Slider, Value-Label)

### Server/GUIServer.py
- **Zeile 355-378:** Erweiterte `handle_speed_command()` Funktion
  - Parsing von `setSpeed:XX` Commands
  - Validation und Clamping
  - Forwarding zu Move-Modul

### Server/Move.py
- **Zeile 1522:** `movement_speed = 35` globale Variable
- **Zeile 1527-1541:** `set_movement_speed()` Setter-Funktion
- **Zeile 1570-1576:** `handle_direction_movement()` verwendet `movement_speed`
- **Zeile 1583-1588:** `handle_turn_movement()` verwendet `movement_speed`
- **Zeile 1609-1619:** `move_thread()` verwendet `movement_speed`

## Lessons Learned

1. **Echtzeit-Kontrolle ist wichtig:**
   - Geschwindigkeit während der Bewegung ändern zu können ist sehr nützlich
   - Benutzer können sofort Feedback geben und anpassen

2. **Validation auf mehreren Ebenen:**
   - GUI limitiert Range (10-60)
   - Server clampt zusätzlich (Defense in Depth)
   - Move-Modul validiert nochmals (Robustheit)

3. **Visuelles Feedback:**
   - Value-Label neben Slider ist wichtig
   - Benutzer sehen sofort, welcher Wert eingestellt ist

4. **Sinnvolle Defaults:**
   - 35 ist ein guter Kompromiss zwischen Speed und Stabilität
   - Range 10-60 deckt alle Anwendungsfälle ab

5. **Backwards Compatibility:**
   - Alte CMD_FAST/CMD_SLOW Commands bleiben unterstützt
   - Kein Breaking Change für bestehende Clients

## Verwandte Features

- **FT43:** Fix Turn Movement with Tripod Gait (verwendet jetzt dynamische Speed)
- **FT44:** Responsive Stop at End of Step (profitiert von langsamen Speeds)
- **FT47:** Refactoring Movement Functions (saubere Struktur für Speed-Integration)

## Zukünftige Erweiterungen

Mögliche Verbesserungen:
1. **Speed-Presets:** Buttons für "Slow" (20), "Normal" (35), "Fast" (50)
2. **Keyboard Shortcuts:** +/- Tasten für Speed-Änderung
3. **Speed-Memory:** Letzten Wert beim GUI-Neustart wiederherstellen
4. **Separate Turn-Speed:** Unterschiedliche Geschwindigkeiten für Forward/Turn
5. **Battery-Compensation:** Automatische Geschwindigkeitsanpassung bei schwacher Batterie

---

**Status:** ✅ Abgeschlossen
**Branch:** master
**Commits:** (wird beim Commit ergänzt)
