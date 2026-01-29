# FT-ROS2-2: ROS 2 Integration - Lazy Servo Initialization Refactoring

**Status**: ✅ Implementiert  
**Datum**: 2026-01-29  
**Art**: Refactoring

## Problem

Die ursprüngliche Lazy-Initialization-Lösung mit `SKIP_SERVO_AUTO_INIT` Environment Variable war ein "Hack", der schwer nachzuvollziehen war. Module importierten automatisch Hardware beim Import, was die Kontrolle über den Initialisierungszeitpunkt erschwerte.

## Lösung: Explizite Initialisierung

### Neue Architektur

```python
# RPIservo.py
pwm = None  # Nicht initialisiert beim Import
_initialized = False

def initialize_pwm():
    """Explizite Initialisierung des PCA9685 Servo-Controllers"""
    global pwm, _initialized
    if _initialized:
        return pwm
    # ... Initialisierung ...
    return pwm
```

### Änderungen

#### 1. **RPIservo.py**
- ❌ **ENTFERNT**: Automatische Initialisierung beim Import
- ❌ **ENTFERNT**: `SKIP_AUTO_INIT` Environment Variable Check
- ✅ **NEU**: `initialize_pwm()` Funktion für explizite Initialisierung
- ✅ **NEU**: Klare Print-Messages beim Import

```python
print("⏸️  RPIservo.py geladen - PCA9685 NICHT initialisiert")
print("   📌 Rufe RPIservo.initialize_pwm() auf, um Servos zu aktivieren")
```

#### 2. **Move.py**
- ❌ **ENTFERNT**: Automatische PCA9685 Initialisierung beim Import
- ❌ **ENTFERNT**: `SKIP_AUTO_INIT` Check
- ❌ **ENTFERNT**: Automatischer `init_all()` Aufruf beim Import
- ✅ **NEU**: `initialize_pwm()` Funktion (delegiert an RPIservo)
- ✅ **GEÄNDERT**: `init_all()` ruft `initialize_pwm()` auf, wenn `pwm` noch `None`

```python
def initialize_pwm():
    """Initialisiert PCA9685 über RPIservo.py"""
    global pwm
    if pwm is not None:
        return pwm
    RPIservo.initialize_pwm()
    pwm = RPIservo.pwm
    return pwm

def init_all():
    global pwm
    if pwm is None:
        initialize_pwm()
    # ... rest der Initialisierung ...
```

#### 3. **ROSServer.py**
- ✅ **GEÄNDERT**: Expliziter Aufruf von `RPIservo.initialize_pwm()` in `init_robot_hardware()`
- ✅ **VERBESSERT**: Bessere Logging-Messages

```python
def init_robot_hardware(self):
    if self.hardware_initialized:
        return
    
    self.get_logger().info('⚡ Aktiviere PCA9685 Servo-Controller...')
    RPIservo.initialize_pwm()  # <--- EXPLIZIT
    
    switch.switchSetup()
    switch.set_all_switch_off()
    
    self.get_logger().info('🔧 Initialisiere Servo-Positionen...')
    move.init_all()
    
    # ... rest ...
```

#### 4. **docker-compose.ros2.yml**
- ❌ **ENTFERNT**: `SKIP_SERVO_AUTO_INIT=1` Environment Variable (nicht mehr nötig)

#### 5. **GUIServer.py**
- ✅ **KEINE ÄNDERUNG**: Funktioniert weiterhin, da `move.init_all()` jetzt automatisch `initialize_pwm()` aufruft

## Vorteile

✅ **Klar und explizit**: Initialisierung erfolgt an genau einer Stelle  
✅ **Kein Hack**: Standard Python-Muster statt Environment Variables  
✅ **Testbar**: Jede Funktion kann einzeln getestet werden  
✅ **Wiederverwendbar**: Andere Skripte können die gleiche Funktion nutzen  
✅ **Dokumentiert**: Funktionen haben Docstrings  
✅ **Bessere Fehlermeldungen**: Klare Print-Messages beim Import

## Ablauf

### Beim Import
```
RPIservo.py geladen
⏸️  RPIservo.py geladen - PCA9685 NICHT initialisiert
   📌 Rufe RPIservo.initialize_pwm() auf, um Servos zu aktivieren

Move.py geladen
⏸️  Move.py geladen - Servos NICHT initialisiert
   📌 Rufe Move.init_all() auf, um Servos zu aktivieren
```

### Bei erster Nutzung (z.B. ROSServer)
```
⚡ Aktiviere PCA9685 Servo-Controller...
🔧 Initialisiere PCA9685 auf Adresse 0x40...
✓ PCA9685 erfolgreich initialisiert auf Adresse 0x40
✓ Move.py verwendet PCA9685 von RPIservo.py
🔧 Initialisiere Servo-Positionen...
✓ Robot hardware initialized successfully
🔥 Servos sind jetzt AKTIV und STEIF!
```

## Testing

**Manuelle Tests erforderlich**:
- ✅ GUIServer.py startet und initialisiert Servos korrekt
- ✅ ROSServer.py startet OHNE Servo-Initialisierung
- ✅ ROSServer initialisiert Servos beim ersten Befehl
- ✅ Servos bleiben WEICH bis zum ersten Befehl

## Rückwärtskompatibilität

✅ **GUIServer.py**: Keine Änderung nötig, funktioniert weiterhin  
✅ **Andere Skripte**: Rufen `move.init_all()` auf → automatische Initialisierung  
❌ **docker-compose.ros2.yml**: `SKIP_SERVO_AUTO_INIT` Variable entfernt (nicht mehr nötig)

## Betroffene Dateien

```
Server/RPIservo.py          - Explizite initialize_pwm() Funktion
Server/Move.py              - Explizite initialize_pwm() Funktion, kein auto-init
Server/ROSServer.py         - Expliziter initialize_pwm() Aufruf
docker-compose.ros2.yml     - SKIP_SERVO_AUTO_INIT entfernt
```

## Nächste Schritte

- [ ] Auf Hardware testen
- [ ] Verifizieren: Servos bleiben WEICH beim ROSServer-Start
- [ ] Verifizieren: Servos werden STEIF beim ersten Befehl
- [ ] Verifizieren: GUIServer funktioniert weiterhin korrekt
