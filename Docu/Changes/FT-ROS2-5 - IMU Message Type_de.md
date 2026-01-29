# FT-ROS2-5: IMU Message Type für Gyro-Daten

**Status**: ✅ Implementiert  
**Datum**: 2026-01-29  
**Art**: Feature / Improvement

## Problem

Die Gyro-Daten wurden als **String** veröffentlicht:

```python
# VORHER (nicht optimal):
gyro_msg = String()
gyro_msg.data = 'G:1.23,0.45,-0.12 A:0.15,0.08,9.81'  # ❌ Muss geparst werden!
```

**Nachteile:**
- ❌ Nutzer muss String parsen
- ❌ Keine Type-Safety
- ❌ Fehleranfällig (Parsing-Fehler)
- ❌ Nicht ROS2-Standard-konform
- ❌ Keine automatische Visualisierung in Tools (RViz, PlotJuggler)
- ❌ Keine automatische Unit-Konvertierung

## Lösung: sensor_msgs/Imu verwenden

ROS2 hat einen **Standard Message Type** für IMU-Daten:

```
sensor_msgs/Imu
```

**Struktur:**
```
std_msgs/Header header
  builtin_interfaces/Time stamp
  string frame_id

geometry_msgs/Quaternion orientation
  float64 x
  float64 y
  float64 z
  float64 w

geometry_msgs/Vector3 angular_velocity
  float64 x  # rad/s
  float64 y  # rad/s
  float64 z  # rad/s

geometry_msgs/Vector3 linear_acceleration
  float64 x  # m/s²
  float64 y  # m/s²
  float64 z  # m/s²

float64[9] orientation_covariance
float64[9] angular_velocity_covariance
float64[9] linear_acceleration_covariance
```

## Implementierung

### 1. Neuer Publisher

```python
# /raspclaws/imu - Strukturierte IMU-Daten (PREFERRED)
self.imu_pub = self.create_publisher(
    Imu,
    '/raspclaws/imu',
    self.qos_profile
)

# /raspclaws/gyro_data - String (DEPRECATED, für Rückwärtskompatibilität)
self.gyro_pub = self.create_publisher(
    String,
    '/raspclaws/gyro_data',
    self.qos_profile
)
```

### 2. IMU Message erstellen und veröffentlichen

```python
# Parse gyro_data string: "G:x,y,z A:x,y,z"
parts = gyro_data.split(' ')
gyro_part = parts[0].split(':')[1]  # "x,y,z"
accel_part = parts[1].split(':')[1]  # "x,y,z"

gyro_values = [float(x) for x in gyro_part.split(',')]
accel_values = [float(x) for x in accel_part.split(',')]

# Create IMU message
imu_msg = Imu()

# Header (Timestamp + Frame)
imu_msg.header.stamp = self.get_clock().now().to_msg()
imu_msg.header.frame_id = 'imu_link'

# Angular velocity (rad/s) - convert from deg/s
imu_msg.angular_velocity.x = gyro_values[0] * 0.017453293  # deg to rad
imu_msg.angular_velocity.y = gyro_values[1] * 0.017453293
imu_msg.angular_velocity.z = gyro_values[2] * 0.017453293

# Linear acceleration (m/s²) - already in m/s²
imu_msg.linear_acceleration.x = accel_values[0]
imu_msg.linear_acceleration.y = accel_values[1]
imu_msg.linear_acceleration.z = accel_values[2]

# Orientation (not available from MPU6050 - set to unknown)
imu_msg.orientation.x = 0.0
imu_msg.orientation.y = 0.0
imu_msg.orientation.z = 0.0
imu_msg.orientation.w = 0.0  # Invalid quaternion = orientation unknown

# Covariance (set first element to -1 to indicate unknown)
imu_msg.orientation_covariance[0] = -1.0
imu_msg.angular_velocity_covariance[0] = 0.0  # Assume known
imu_msg.linear_acceleration_covariance[0] = 0.0  # Assume known

self.imu_pub.publish(imu_msg)
```

### 3. Unit-Konvertierung

**MPU6050 liefert:**
- Gyro: **degrees/sec**
- Accel: **m/s²**

**ROS2 Standard erfordert:**
- Gyro: **rad/s**
- Accel: **m/s²**

**Konvertierung:**
```python
rad_s = deg_s * 0.017453293  # π/180
```

### 4. Orientation Handling

MPU6050 liefert **keine Orientation** (nur Gyro + Accel, kein Magnetometer).

**Lösung:**
```python
# Invalid quaternion = orientation unknown
imu_msg.orientation.x = 0.0
imu_msg.orientation.y = 0.0
imu_msg.orientation.z = 0.0
imu_msg.orientation.w = 0.0

# Covariance = -1 bedeutet "unknown"
imu_msg.orientation_covariance[0] = -1.0
```

Dies ist **ROS2-konform** und signalisiert: "Orientation ist nicht verfügbar".

## Neue Topics

| Topic | Message Type | Beschreibung | Status |
|-------|--------------|--------------|--------|
| `/raspclaws/imu` | `sensor_msgs/Imu` | Strukturierte IMU-Daten | ✅ **PREFERRED** |
| `/raspclaws/gyro_data` | `std_msgs/String` | String-Format | ⚠️ **DEPRECATED** |

**Migration-Path:**
- **Neu**: Nutze `/raspclaws/imu`
- **Alt**: `/raspclaws/gyro_data` bleibt für Rückwärtskompatibilität

## Verwendung

### Strukturierte IMU-Daten empfangen

```bash
# IMU-Daten als strukturierte Message (PREFERRED):
ros2 topic echo /raspclaws/imu

# Ausgabe:
header:
  stamp:
    sec: 1706533920
    nanosec: 123456789
  frame_id: imu_link
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 0.0
angular_velocity:
  x: 0.0215  # rad/s (converted from deg/s)
  y: 0.0079
  z: -0.0021
linear_acceleration:
  x: 0.15    # m/s²
  y: 0.08
  z: 9.81
orientation_covariance: [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
angular_velocity_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
linear_acceleration_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

### String-Format (DEPRECATED)

```bash
# String-Format (für Rückwärtskompatibilität):
ros2 topic echo /raspclaws/gyro_data

# Ausgabe:
data: 'G:1.23,0.45,-0.12 A:0.15,0.08,9.81'
```

## Vorteile

### 1. **Automatische Visualisierung in RViz**

```bash
# RViz kann IMU-Daten direkt darstellen:
rviz2

# Add Display → By Topic → /raspclaws/imu → Imu
```

**RViz zeigt:**
- Acceleration Arrows (Beschleunigung)
- Angular Velocity Arrows (Drehgeschwindigkeit)
- Automatische Frame-Transformation

### 2. **PlotJuggler Integration**

```bash
# PlotJuggler kann IMU-Daten direkt plotten:
ros2 run plotjuggler plotjuggler

# Drag & Drop:
# - /raspclaws/imu/angular_velocity/x
# - /raspclaws/imu/angular_velocity/y
# - /raspclaws/imu/linear_acceleration/z
```

**Kein Parsing nötig!**

### 3. **Python-Code (Clean)**

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuListener(Node):
    def __init__(self):
        super().__init__('imu_listener')
        self.subscription = self.create_subscription(
            Imu,
            '/raspclaws/imu',
            self.imu_callback,
            10)
    
    def imu_callback(self, msg):
        # Direkter Zugriff auf strukturierte Daten - KEIN PARSING!
        gyro_x = msg.angular_velocity.x  # rad/s
        accel_z = msg.linear_acceleration.z  # m/s²
        
        print(f"Gyro X: {gyro_x:.3f} rad/s")
        print(f"Accel Z: {accel_z:.2f} m/s²")
```

**VORHER (mit String):**
```python
# String-Parsing - fehleranfällig!
parts = msg.data.split(' ')
gyro_part = parts[0].split(':')[1]
gyro_values = [float(x) for x in gyro_part.split(',')]
gyro_x = gyro_values[0] * 0.017453293  # Manuell konvertieren!
```

### 4. **Type-Safety**

```python
# Type-Checker (mypy) erkennt Fehler:
imu_msg.angular_velocity.x = "abc"  # ❌ Type Error!
imu_msg.angular_velocity.x = 1.23   # ✅ Korrekt
```

### 5. **Standard ROS2 Tools**

Alle ROS2-Tools arbeiten direkt mit `sensor_msgs/Imu`:
- `ros2 bag` - Aufzeichnung
- `ros2 topic hz` - Frequenz
- `ros2 topic bw` - Bandbreite
- `rqt_plot` - Echtzeit-Plotting
- `robot_state_publisher` - TF-Integration

## Vergleich

| Feature | String (`gyro_data`) | Structured (`imu`) |
|---------|---------------------|-------------------|
| Parsing nötig | ✅ Ja, fehleranfällig | ❌ Nein |
| Type-Safety | ❌ Nein | ✅ Ja |
| RViz Visualisierung | ❌ Nein | ✅ Ja |
| PlotJuggler | ❌ Manuell parsen | ✅ Direkt |
| Unit-Konvertierung | ❌ Manuell | ✅ Automatisch (Konvention) |
| Timestamp | ❌ Fehlt | ✅ Header.stamp |
| Frame-ID | ❌ Fehlt | ✅ Header.frame_id |
| Standard-konform | ❌ Nein | ✅ Ja (ROS2 REP 145) |
| Code-Qualität | ⚠️ String-Parsing | ✅ Clean |

## Migration

### Alt (DEPRECATED):

```python
# Subscriber für String-Format
self.subscription = self.create_subscription(
    String,
    '/raspclaws/gyro_data',
    self.gyro_callback,
    10)

def gyro_callback(self, msg):
    # Manuelles Parsing
    parts = msg.data.split(' ')
    # ...
```

### Neu (RECOMMENDED):

```python
# Subscriber für IMU-Format
self.subscription = self.create_subscription(
    Imu,
    '/raspclaws/imu',
    self.imu_callback,
    10)

def imu_callback(self, msg):
    # Direkter Zugriff
    gyro_x = msg.angular_velocity.x  # rad/s
    accel_z = msg.linear_acceleration.z  # m/s²
```

## Rückwärtskompatibilität

**Beide Topics werden veröffentlicht:**

```python
# BEIDE werden published:
self.gyro_pub.publish(gyro_msg)      # String (alt)
self.imu_pub.publish(imu_msg)        # IMU (neu)
```

**Nutzer können schrittweise migrieren:**
1. Alte Clients nutzen weiter `/raspclaws/gyro_data`
2. Neue Clients nutzen `/raspclaws/imu`
3. Später: `/raspclaws/gyro_data` entfernen (Breaking Change in v2.0)

## Standards & Konventionen

### ROS2 REP 145: IMU Frames

**Frame-ID**: `imu_link`

**Konvention:**
- X-Achse: nach vorne (Forward)
- Y-Achse: nach links (Left)
- Z-Achse: nach oben (Up)

**Right-Handed Coordinate System**

### Unit-Standards

| Sensor | ROS2 Standard | MPU6050 Output | Konvertierung |
|--------|---------------|----------------|---------------|
| Gyro | rad/s | deg/s | `rad = deg * π/180` |
| Accel | m/s² | m/s² | Keine |
| Orientation | Quaternion | N/A | Nicht verfügbar |

## Testing

### 1. Topic verfügbar?

```bash
ros2 topic list | grep imu
# /raspclaws/imu
# /raspclaws/gyro_data  (deprecated)
```

### 2. Message Type prüfen

```bash
ros2 topic type /raspclaws/imu
# sensor_msgs/msg/Imu

ros2 interface show sensor_msgs/msg/Imu
# (zeigt Struktur)
```

### 3. Daten empfangen

```bash
# Strukturiert (NEU):
ros2 topic echo /raspclaws/imu

# String (ALT):
ros2 topic echo /raspclaws/gyro_data
```

### 4. Publish-Rate

```bash
ros2 topic hz /raspclaws/imu
# average rate: 0.500 (alle 2 Sekunden)
```

### 5. RViz Visualisierung

```bash
rviz2

# Add Display:
# - Type: Imu
# - Topic: /raspclaws/imu
# - Fixed Frame: imu_link (oder map)
```

### 6. PlotJuggler

```bash
ros2 run plotjuggler plotjuggler

# Data Source: ROS 2 Topic Subscriber
# Topic: /raspclaws/imu
# Plot:
#   - angular_velocity.x
#   - angular_velocity.y
#   - linear_acceleration.z
```

## Error Handling

### Sensor nicht verfügbar

```bash
# IMU Topic zeigt keine Daten
# ODER String-Topic zeigt:
data: 'MPU:N/A'

# Logs prüfen:
docker compose logs -f | grep -i mpu
```

### Parse-Fehler

```python
# Wenn String-Parsing fehlschlägt:
try:
    parts = gyro_data.split(' ')
    # ...
except Exception as parse_error:
    self.get_logger().warn(f'Failed to parse gyro_data for IMU message: {parse_error}')
```

**Verhalten:**
- String-Topic funktioniert weiter (`gyro_data`)
- IMU-Topic wird nicht veröffentlicht (nur bei Parse-Fehler)
- Logs zeigen Warning

## Betroffene Dateien

```
Server/ROSServer.py                                    - IMU Publisher + Message-Erstellung
Docu/Changes/FT-ROS2-5 - IMU Message Type_de.md       - Diese Dokumentation
show_imu_message.sh                                    - Zeigt IMU Message-Struktur
```

## Zukünftige Verbesserungen (Optional)

### 1. Orientation von Sensor Fusion

Nutze `imu_complementary_filter` oder `robot_localization`:

```bash
# Berechne Orientation aus Gyro + Accel:
ros2 run imu_complementary_filter complementary_filter_node
```

### 2. Magnetometer hinzufügen

MPU9250 (statt MPU6050) hat Magnetometer → Absolute Orientation möglich

### 3. TF-Integration

```python
# Publish IMU als TF-Transform:
broadcaster = TransformBroadcaster(self)
# ...
```

## Zusammenfassung

**Vorher**: String-Format → Parsing nötig, fehleranfällig, nicht standard-konform

**Nachher**: 
- ✅ **`/raspclaws/imu`** (sensor_msgs/Imu) - Strukturiert, standard-konform, tool-kompatibel
- ⚠️ **`/raspclaws/gyro_data`** (String) - Deprecated, aber für Rückwärtskompatibilität erhalten

**Empfehlung**: Nutze `/raspclaws/imu` für alle neuen Projekte!
