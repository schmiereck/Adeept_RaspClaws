# FT-ROS2-4: Servo Standby/Wakeup Service

**Status**: ✅ Implementiert  
**Datum**: 2026-01-29  
**Art**: Feature

## Übersicht

ROS2 Service zum Versetzen der Servos in den Standby-Modus (soft, low power) oder zum Aufwecken (stand position).

## Problem

Der ROSServer hatte keine Möglichkeit, die Servos in den Standby-Modus zu versetzen, um:
- Strom zu sparen (wichtig bei Akku-Betrieb)
- Servos zu entspannen (keine dauerhafte mechanische Belastung)
- Roboter manuell zu bewegen (z.B. für Transport oder Tests)

## Lösung

### Neuer ROS2 Service

| Service | Type | Beschreibung |
|---------|------|--------------|
| `/raspclaws/set_servo_standby` | `std_srvs/SetBool` | Servo Standby ein/aus |

**Request:**
- `data: true` → Servos in STANDBY versetzen (weich, kein Strom)
- `data: false` → Servos WAKEUP (stand position, bereit)

**Response:**
- `success: bool` → True wenn erfolgreich
- `message: string` → Statusmeldung

## Verwendung

### Servos in Standby versetzen

```bash
# Auf PC oder Pi (mit ROS2):
ros2 service call /raspclaws/set_servo_standby std_srvs/srv/SetBool "{data: true}"

# Erwartete Ausgabe:
# success: True
# message: 'Servos in STANDBY mode - legs are soft, low power'
```

**Effekt auf dem Roboter:**
1. Alle Bewegungs-Threads werden pausiert
2. PWM-Signale werden auf 0 gesetzt
3. Servos werden "weich" und können manuell bewegt werden
4. Stromverbrauch sinkt deutlich
5. Status-Topic zeigt: `Servos: STANDBY`

### Servos aufwecken

```bash
ros2 service call /raspclaws/set_servo_standby std_srvs/srv/SetBool "{data: false}"

# Erwartete Ausgabe:
# success: True
# message: 'Servos WAKEUP - robot ready in stand position'
```

**Effekt auf dem Roboter:**
1. Servos fahren in Stand-Position (sicher, bekannt)
2. PWM-Signale werden aktiviert
3. Servos werden "steif" und halten Position
4. Bewegungs-Threads werden fortgesetzt
5. Status-Topic zeigt: `Servos: ACTIVE`
6. Roboter ist bereit für Bewegungen

## Status-Monitoring

Der Servo-Standby-Status wird im `/raspclaws/status` Topic veröffentlicht:

```bash
# Status ansehen:
ros2 topic echo /raspclaws/status

# Beispiel-Ausgaben:
# data: 'HW_READY - Servos: ACTIVE, Smooth: False, SmoothCam: False'
# data: 'HW_READY - Servos: STANDBY, Smooth: False, SmoothCam: False'
# data: 'HW_LAZY - Servos: ACTIVE, Smooth: False, SmoothCam: False'
```

**Status-Felder:**
- `HW_READY`: Hardware initialisiert (nach erstem Befehl)
- `HW_LAZY`: Hardware noch nicht initialisiert
- `Servos: ACTIVE`: Servos steif, bereit für Bewegung
- `Servos: STANDBY`: Servos weich, Strom sparen

## Implementierung

### 1. State Variable hinzugefügt

```python
class RaspClawsNode(Node):
    def __init__(self):
        # ...
        self.smooth_mode = False
        self.smooth_cam_mode = False
        self.servo_standby_active = False  # ✅ NEU
```

### 2. Service erstellt

```python
def create_services(self):
    # ...
    # Servo standby/wakeup service
    self.servo_standby_srv = self.create_service(
        SetBool,
        '/raspclaws/set_servo_standby',
        self.set_servo_standby_callback
    )
```

### 3. Service Callback implementiert

```python
def set_servo_standby_callback(self, request, response):
    """Service callback to set servo standby mode"""
    try:
        standby_requested = request.data  # True = Standby, False = Wakeup

        if not ROBOT_MODULES_AVAILABLE:
            # MOCK MODE
            self.servo_standby_active = standby_requested
            response.success = True
            response.message = f'MOCK: Servo standby set to {standby_requested}'
            return response

        # Lazy initialization: Initialize hardware before standby/wakeup
        if not self.hardware_initialized:
            self.init_robot_hardware()

        if standby_requested:
            # STANDBY: Put servos to sleep (soft, low power)
            self.get_logger().info('🔋 SERVO STANDBY - Stopping PWM signals')
            move.standby()  # Nutzt Move.standby() wie GUIServer
            self.servo_standby_active = True
            response.success = True
            response.message = 'Servos in STANDBY mode - legs are soft, low power'
        else:
            # WAKEUP: Restore servos to stand position
            self.get_logger().info('⚡ SERVO WAKEUP - Restoring servo positions')
            move.wakeup()  # Nutzt Move.wakeup() wie GUIServer
            self.servo_standby_active = False
            response.success = True
            response.message = 'Servos WAKEUP - robot ready in stand position'

        self.get_logger().info(f'Servo standby mode: {self.servo_standby_active}')

    except Exception as e:
        self.get_logger().error(f'Error setting servo standby mode: {e}')
        response.success = False
        response.message = f'Error: {e}'

    return response
```

### 4. Status-Topic angepasst

```python
def publish_system_info(self):
    # ...
    # Status message
    status_msg = String()
    hw_status = "HW_READY" if self.hardware_initialized else "HW_LAZY"
    servo_status = "STANDBY" if self.servo_standby_active else "ACTIVE"  # ✅ NEU
    status_msg.data = f'{hw_status} - Servos: {servo_status}, Smooth: {self.smooth_mode}, SmoothCam: {self.smooth_cam_mode}'
    self.status_pub.publish(status_msg)
```

## Vergleich mit GUIServer

Der ROSServer nutzt die **gleichen Funktionen** wie der GUIServer:

| Funktion | GUIServer | ROSServer |
|----------|-----------|-----------|
| Standby | ✅ `move.standby()` | ✅ `move.standby()` |
| Wakeup | ✅ `move.wakeup()` | ✅ `move.wakeup()` |
| Status | ✅ `servo_standby_active` | ✅ `self.servo_standby_active` |

## Move.py Funktionen (zur Info)

### move.standby()

```python
def standby():
    """
    Put all servos into standby mode - stops PWM signals.
    Servos become 'soft' and can be moved by hand.
    Saves power while keeping the Pi running.
    """
    global move_stu
    move_stu = 0  # Stop any ongoing movement

    # Pause the RobotM thread to stop all movement
    rm.pause()

    # Stop PWM signals on all channels
    for channel in range(16):
        pwm.set_pwm(channel, 0, 0)
```

### move.wakeup()

```python
def wakeup():
    """
    Wake up servos from standby - moves servos to stand position.
    """
    global direction_command, turn_command, step_set, move_stu

    # Reset movement commands to safe defaults
    direction_command = MOVE_STAND
    turn_command = MOVE_NO
    step_set = 1
    move_stu = 1

    # Move servos to stand position (safe, known position)
    stand()

    # Reset to 'no' after stand is complete
    direction_command = MOVE_NO

    # Resume the RobotM thread
    rm.resume()
```

## Lazy Initialization

Der Service funktioniert auch **vor** der Hardware-Initialisierung:

```python
# Lazy initialization: Initialize hardware before standby/wakeup
if not self.hardware_initialized:
    self.init_robot_hardware()
```

**Ablauf:**
1. ROSServer startet → Hardware NICHT initialisiert
2. Service-Call: `set_servo_standby false` → Hardware wird initialisiert → Wakeup
3. Servos fahren in Stand-Position
4. Roboter ist bereit

## Use Cases

### 1. Strom sparen bei Pause

```bash
# Roboter ist bereit, aber Pause → Strom sparen
ros2 service call /raspclaws/set_servo_standby std_srvs/srv/SetBool "{data: true}"

# ... Pause ...

# Weiter arbeiten
ros2 service call /raspclaws/set_servo_standby std_srvs/srv/SetBool "{data: false}"
```

### 2. Roboter manuell bewegen

```bash
# Servos weich machen
ros2 service call /raspclaws/set_servo_standby std_srvs/srv/SetBool "{data: true}"

# Jetzt Roboter manuell bewegen (z.B. für Transport)

# Servos wieder aktivieren (Stand-Position)
ros2 service call /raspclaws/set_servo_standby std_srvs/srv/SetBool "{data: false}"
```

### 3. Notfall-Stop

```bash
# Bei Problemen: Sofort alle Servos stoppen
ros2 service call /raspclaws/set_servo_standby std_srvs/srv/SetBool "{data: true}"
```

## MOCK MODE

Wenn `ROBOT_MODULES_AVAILABLE = False` (z.B. auf PC):
- Service funktioniert trotzdem
- State wird gesetzt (`self.servo_standby_active`)
- Status-Topic wird aktualisiert
- Keine Hardware-Aufrufe

```bash
# Auf PC (ohne Hardware):
ros2 service call /raspclaws/set_servo_standby std_srvs/srv/SetBool "{data: true}"

# Response:
# success: True
# message: 'MOCK: Servo standby set to True'

# Status-Topic:
ros2 topic echo /raspclaws/status
# data: 'MOCK MODE - Servos: STANDBY, Smooth: False, SmoothCam: False'
```

## Error Handling

```python
try:
    # Service logic
    if standby_requested:
        move.standby()
    else:
        move.wakeup()
    response.success = True
except Exception as e:
    self.get_logger().error(f'Error setting servo standby mode: {e}')
    response.success = False
    response.message = f'Error: {e}'
```

Bei Fehlern:
- `success = False`
- `message` enthält Fehlerdetails
- Exception wird geloggt
- Kein Crash

## Integration mit anderen Services

Servo Standby ist **unabhängig** von anderen Modi:

```bash
# Smooth Mode ein
ros2 service call /raspclaws/set_smooth_mode std_srvs/srv/SetBool "{data: true}"

# SmoothCam ein
ros2 service call /raspclaws/set_smooth_cam std_srvs/srv/SetBool "{data: true}"

# Servos in Standby (Modi bleiben erhalten)
ros2 service call /raspclaws/set_servo_standby std_srvs/srv/SetBool "{data: true}"

# Status zeigt alles:
# data: 'HW_READY - Servos: STANDBY, Smooth: True, SmoothCam: True'

# Wakeup → Modi sind noch aktiv
ros2 service call /raspclaws/set_servo_standby std_srvs/srv/SetBool "{data: false}"
```

## Betroffene Dateien

```
Server/ROSServer.py     - Service + Callback + State + Status-Update
```

## Testing

### 1. Service verfügbar?

```bash
ros2 service list | grep servo_standby
# /raspclaws/set_servo_standby
```

### 2. Service-Typ korrekt?

```bash
ros2 service type /raspclaws/set_servo_standby
# std_srvs/srv/SetBool
```

### 3. Standby Test

```bash
# Standby aktivieren
ros2 service call /raspclaws/set_servo_standby std_srvs/srv/SetBool "{data: true}"

# Status prüfen
ros2 topic echo /raspclaws/status --once
# data: 'HW_READY - Servos: STANDBY, ...'

# Hardware prüfen: Servos sollten WEICH sein (manuell bewegbar)
```

### 4. Wakeup Test

```bash
# Wakeup
ros2 service call /raspclaws/set_servo_standby std_srvs/srv/SetBool "{data: false}"

# Status prüfen
ros2 topic echo /raspclaws/status --once
# data: 'HW_READY - Servos: ACTIVE, ...'

# Hardware prüfen: Servos sollten STEIF sein (Stand-Position halten)
```

## Vorteile

✅ **Stromersparnis**: Deutlich weniger Verbrauch im Standby  
✅ **Servo-Schonung**: Keine dauerhafte mechanische Belastung  
✅ **Flexibilität**: Roboter manuell bewegbar  
✅ **Sicherheit**: Schneller Notfall-Stop möglich  
✅ **ROS2-Standard**: `SetBool` Service (wie andere Services)  
✅ **Status-Transparenz**: Immer sichtbar im Status-Topic  
✅ **Konsistenz**: Gleiche Funktionen wie GUIServer  
✅ **Robust**: Error Handling, MOCK MODE Support

## Zusammenfassung

Der ROSServer kann jetzt die Servos in den **Standby-Modus** versetzen, um Strom zu sparen und die Servos zu schonen. Der Service nutzt die gleichen bewährten Funktionen wie der GUIServer (`move.standby()`, `move.wakeup()`) und ist vollständig in das Status-Topic integriert.
