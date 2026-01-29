# FT-ROS2-3: ROSServer Status-Daten Integration

**Status**: ✅ Implementiert  
**Datum**: 2026-01-29  
**Art**: Feature / Bugfix

## Problem

Der ROSServer veröffentlichte keine echten Status-Daten über ROS2 Topics:
- ❌ Battery Voltage war hardcoded auf `7.4`
- ❌ CPU Usage war hardcoded auf `25.0`
- ❌ Keine CPU Temperatur
- ❌ Keine RAM Usage
- ❌ Keine Servo-Positionen
- ❌ Keine MPU6050 Gyro-Daten
- ❌ Info.py wurde nicht importiert

## Lösung

### Integration mit Info.py

Der ROSServer nutzt jetzt die gleichen Funktionen wie der GUIServer:

```python
import Info

# Echte Daten von Hardware-Sensoren:
battery_voltage = Info.get_battery_voltage()      # Von ADS7830
cpu_temp = Info.get_cpu_tempfunc()                # Von thermal_zone0
cpu_usage = Info.get_cpu_use()                    # Von psutil
ram_usage = Info.get_ram_info()                   # Von psutil
```

### Neue ROS2 Topics

| Topic | Message Type | Beschreibung | Quelle |
|-------|--------------|--------------|--------|
| `/raspclaws/battery` | `std_msgs/Float32` | Batteriespannung in Volt | ADS7830 (Info.py) |
| `/raspclaws/cpu_temp` | `std_msgs/Float32` | CPU Temperatur in °C | thermal_zone0 |
| `/raspclaws/cpu_usage` | `std_msgs/Float32` | CPU Auslastung 0-100% | psutil |
| `/raspclaws/ram_usage` | `std_msgs/Float32` | RAM Auslastung 0-100% | psutil |
| `/raspclaws/servo_positions` | `std_msgs/String` | Servo-Positionen | Move.py |
| `/raspclaws/gyro_data` | `std_msgs/String` | MPU6050 Gyro/Accel | Move.py |
| `/raspclaws/status` | `std_msgs/String` | Status-Nachrichten | ROSServer |

### Code-Änderungen

#### 1. Info.py importieren

```python
# ROSServer.py
try:
    import Move as move
    import RPIservo
    import Switch as switch
    import RobotLight as robotLight
    import Info  # ✅ NEU
    ROBOT_MODULES_AVAILABLE = True
except ImportError as e:
    # ...
```

#### 2. Neue Publisher erstellen

```python
def create_publishers(self):
    # Battery voltage
    self.battery_pub = self.create_publisher(Float32, '/raspclaws/battery', self.qos_profile)
    
    # CPU temperature ✅ NEU
    self.cpu_temp_pub = self.create_publisher(Float32, '/raspclaws/cpu_temp', self.qos_profile)
    
    # CPU usage (vorher hardcoded, jetzt echt)
    self.cpu_pub = self.create_publisher(Float32, '/raspclaws/cpu_usage', self.qos_profile)
    
    # RAM usage ✅ NEU
    self.ram_pub = self.create_publisher(Float32, '/raspclaws/ram_usage', self.qos_profile)
    
    # Servo positions ✅ NEU
    self.servo_positions_pub = self.create_publisher(String, '/raspclaws/servo_positions', self.qos_profile)
    
    # Gyro data ✅ NEU
    self.gyro_pub = self.create_publisher(String, '/raspclaws/gyro_data', self.qos_profile)
    
    # Status messages
    self.status_pub = self.create_publisher(String, '/raspclaws/status', self.qos_profile)
```

#### 3. Echte Daten veröffentlichen

```python
def publish_system_info(self):
    """Publish system information periodically (every 2 seconds)"""
    
    if not ROBOT_MODULES_AVAILABLE:
        # MOCK MODE: Placeholder-Daten
        # ...
        return
    
    # ==================== REAL DATA ====================
    
    # Battery voltage (from Info.py)
    battery_voltage_str = Info.get_battery_voltage()
    battery_msg = Float32()
    battery_msg.data = float(battery_voltage_str)
    self.battery_pub.publish(battery_msg)
    
    # CPU temperature (from Info.py)
    cpu_temp_str = Info.get_cpu_tempfunc()
    cpu_temp_msg = Float32()
    cpu_temp_msg.data = float(cpu_temp_str)
    self.cpu_temp_pub.publish(cpu_temp_msg)
    
    # CPU usage (from Info.py)
    cpu_use_str = Info.get_cpu_use()
    cpu_msg = Float32()
    cpu_msg.data = float(cpu_use_str)
    self.cpu_pub.publish(cpu_msg)
    
    # RAM usage (from Info.py)
    ram_use_str = Info.get_ram_info()
    ram_msg = Float32()
    ram_msg.data = float(ram_use_str)
    self.ram_pub.publish(ram_msg)
    
    # Servo positions (from Move.py) - only if hardware is initialized
    if self.hardware_initialized:
        servo_positions = move.get_servo_positions_info()
        servo_msg = String()
        servo_msg.data = servo_positions
        self.servo_positions_pub.publish(servo_msg)
    
    # MPU6050 Gyro/Accelerometer data (from Move.py) - only if hardware is initialized
    if self.hardware_initialized:
        gyro_data = move.get_mpu6050_data()
        gyro_msg = String()
        gyro_msg.data = gyro_data
        self.gyro_pub.publish(gyro_msg)
    
    # Status message
    status_msg = String()
    hw_status = "HW_READY" if self.hardware_initialized else "HW_LAZY"
    status_msg.data = f'{hw_status} - Smooth: {self.smooth_mode}, SmoothCam: {self.smooth_cam_mode}'
    self.status_pub.publish(status_msg)
```

### Besonderheiten

#### Lazy Initialization

- **Servo-Positionen** und **Gyro-Daten** werden nur gesendet, wenn Hardware initialisiert ist
- **System-Daten** (Battery, CPU, RAM) werden immer gesendet (auch im Lazy Mode)

```python
if self.hardware_initialized:
    # Nur nach erstem Bewegungsbefehl
    servo_positions = move.get_servo_positions_info()
    gyro_data = move.get_mpu6050_data()
```

#### MOCK MODE

Wenn `ROBOT_MODULES_AVAILABLE = False` (z.B. auf PC):
- Placeholder-Daten werden gesendet
- Keine Fehler, wenn Hardware fehlt

#### Error Handling

```python
try:
    battery_msg.data = float(battery_voltage_str)
except ValueError:
    battery_msg.data = 0.0  # Fallback bei Parse-Fehler
```

## Test

### Topics anzeigen

```bash
# Auf dem Pi oder PC (mit ROS2):
ros2 topic list

# Erwartete Ausgabe:
# /raspclaws/battery
# /raspclaws/cpu_temp
# /raspclaws/cpu_usage
# /raspclaws/ram_usage
# /raspclaws/servo_positions
# /raspclaws/gyro_data
# /raspclaws/status
```

### Echte Daten ansehen

```bash
# Battery Voltage (sollte ~7.4V bei Akku sein)
ros2 topic echo /raspclaws/battery

# CPU Temperatur (sollte 40-60°C sein)
ros2 topic echo /raspclaws/cpu_temp

# CPU Usage (sollte schwanken)
ros2 topic echo /raspclaws/cpu_usage

# RAM Usage (sollte schwanken)
ros2 topic echo /raspclaws/ram_usage

# Status (zeigt HW_LAZY oder HW_READY)
ros2 topic echo /raspclaws/status

# Servo Positionen (nur nach Hardware-Init)
ros2 topic echo /raspclaws/servo_positions

# Gyro Data (nur nach Hardware-Init)
ros2 topic echo /raspclaws/gyro_data
```

### Erwartete Ausgabe (Beispiel)

```bash
$ ros2 topic echo /raspclaws/battery
data: 7.42

$ ros2 topic echo /raspclaws/cpu_temp
data: 52.3

$ ros2 topic echo /raspclaws/cpu_usage
data: 18.5

$ ros2 topic echo /raspclaws/ram_usage
data: 42.1

$ ros2 topic echo /raspclaws/status
data: 'HW_LAZY - Smooth: False, SmoothCam: False'

# Nach erstem Bewegungsbefehl:
$ ros2 topic echo /raspclaws/status
data: 'HW_READY - Smooth: False, SmoothCam: False'

$ ros2 topic echo /raspclaws/servo_positions
data: 'L1:266,310 L2:334,279 L3:266,310 R1:266,321 R2:334,290 R3:266,321'

$ ros2 topic echo /raspclaws/gyro_data
data: 'X:0.12 Y:-0.08 Z:9.81'
```

## Vergleich mit GUIServer

Der ROSServer nutzt jetzt die **gleichen Funktionen** wie der GUIServer:

| Feature | GUIServer | ROSServer (vorher) | ROSServer (jetzt) |
|---------|-----------|-------------------|-------------------|
| CPU Temperatur | ✅ `Info.get_cpu_tempfunc()` | ❌ Fehlt | ✅ `Info.get_cpu_tempfunc()` |
| CPU Usage | ✅ `Info.get_cpu_use()` | ❌ Hardcoded (25.0) | ✅ `Info.get_cpu_use()` |
| RAM Usage | ✅ `Info.get_ram_info()` | ❌ Fehlt | ✅ `Info.get_ram_info()` |
| Battery Voltage | ✅ `Info.get_battery_voltage()` | ❌ Hardcoded (7.4) | ✅ `Info.get_battery_voltage()` |
| Servo Positionen | ✅ `move.get_servo_positions_info()` | ❌ Fehlt | ✅ `move.get_servo_positions_info()` |
| MPU6050 Gyro | ✅ `move.get_mpu6050_data()` | ❌ Fehlt | ✅ `move.get_mpu6050_data()` |

## Betroffene Dateien

```
Server/ROSServer.py     - Info.py Import, neue Topics, echte Daten
```

## Technische Details

### Daten-Quellen

1. **ADS7830** (I2C 0x48): Batteriespannung über Spannungsteiler
2. **thermal_zone0**: CPU Temperatur vom Kernel
3. **psutil**: CPU Usage, RAM Usage
4. **PCA9685** (I2C 0x40): Servo-Positionen (über Move.py)
5. **MPU6050** (I2C 0x68): Gyro/Accelerometer (über Move.py)

### Update-Frequenz

- **System-Daten**: Alle 2 Sekunden (Timer in ROSServer)
- **GUIServer**: Alle 200ms (schneller für Servo-Analyse)

### Fehlerbehandlung

- Parse-Fehler bei Float-Konvertierung → Fallback auf 0.0
- Fehlende Hardware (MOCK MODE) → Placeholder-Daten
- Exception in `publish_system_info()` → Logging, kein Crash

## Vorteile

✅ **Konsistenz**: Gleiche Daten-Quellen wie GUIServer  
✅ **Echte Daten**: Keine Placeholders mehr  
✅ **Vollständig**: Alle wichtigen Metriken verfügbar  
✅ **Robust**: Error Handling bei fehlender Hardware  
✅ **ROS2-Standard**: Standard Message Types (Float32, String)  
✅ **Erweiterbar**: Einfach neue Topics hinzufügen

## Nächste Schritte (Optional)

- [ ] Custom Message Types für strukturierte Daten (z.B. ServoArray, ImuData)
- [ ] Power Management Status als Topics (servo_standby, camera_paused)
- [ ] Camera Feed als Image Topic (FPV.py Integration)
- [ ] Diagnostics Topic für System-Health
