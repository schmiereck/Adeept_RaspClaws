# FT-ROS2-3.1: Gyro-Daten Bugfix - Immer verfügbar

**Status**: ✅ Implementiert  
**Datum**: 2026-01-29  
**Art**: Bugfix / Verbesserung  
**Related**: FT-ROS2-3 (ROSServer Status Integration)

## Problem

Das `/raspclaws/gyro_data` Topic lieferte keine Daten, weil:
- Gyro-Daten wurden nur veröffentlicht, wenn `hardware_initialized == True`
- Das passiert erst nach dem ersten Bewegungsbefehl
- Nutzer erwartete sofort Gyro-Daten (wie bei CPU, Battery)

## Analyse

```python
# VORHER (Buggy):
if self.hardware_initialized:
    servo_positions = move.get_servo_positions_info()
    gyro_data = move.get_mpu6050_data()  # ❌ Nur nach Hardware-Init!
```

**Problem**: Der MPU6050 Gyro-Sensor ist **unabhängig von den Servos**!
- Gyro-Sensor kann auch vor Servo-Initialisierung gelesen werden
- Gyro-Daten sollten wie CPU/Battery-Daten behandelt werden (immer verfügbar)

## Lösung: Option 1 - Gyro-Daten immer veröffentlichen

```python
# NACHHER (Fixed):
# MPU6050 Gyro/Accelerometer data (from Move.py)
# Gyro sensor is independent of servos, so we can publish even before hardware init
try:
    gyro_data = move.get_mpu6050_data()
    gyro_msg = String()
    gyro_msg.data = gyro_data
    self.gyro_pub.publish(gyro_msg)
    self.get_logger().debug(f'Published gyro_data: {gyro_data}')
except Exception as e:
    self.get_logger().error(f'Error getting/publishing gyro data: {e}')

# Servo positions (from Move.py) - only if hardware is initialized
if self.hardware_initialized:
    servo_positions = move.get_servo_positions_info()
    # ...
```

## Änderungen

### 1. Gyro-Daten aus `if self.hardware_initialized` Block herausgenommen

**Vorher**:
```python
if self.hardware_initialized:
    servo_positions = move.get_servo_positions_info()
    gyro_data = move.get_mpu6050_data()  # Zusammen mit Servo-Daten
```

**Nachher**:
```python
# Gyro-Daten IMMER veröffentlichen
gyro_data = move.get_mpu6050_data()

# Servo-Daten NUR nach Hardware-Init
if self.hardware_initialized:
    servo_positions = move.get_servo_positions_info()
```

### 2. Error Handling hinzugefügt

```python
try:
    gyro_data = move.get_mpu6050_data()
    gyro_msg = String()
    gyro_msg.data = gyro_data
    self.gyro_pub.publish(gyro_msg)
    self.get_logger().debug(f'Published gyro_data: {gyro_data}')
except Exception as e:
    self.get_logger().error(f'Error getting/publishing gyro data: {e}')
```

**Vorteile**:
- Kein Crash bei Sensor-Fehlern
- Fehler werden geloggt
- Service läuft weiter

### 3. Debug-Logging hinzugefügt

```python
self.get_logger().debug(f'Published gyro_data: {gyro_data}')
```

**Ermöglicht Debugging** mit:
```bash
# Logs mit Debug-Level:
docker compose -f docker-compose.ros2.yml logs -f | grep gyro_data
```

## Verhalten nach Fix

### Lazy Mode (HW_LAZY - vor erstem Bewegungsbefehl)

| Topic | Status | Wert |
|-------|--------|------|
| `/raspclaws/battery` | ✅ Verfügbar | `7.42` |
| `/raspclaws/cpu_temp` | ✅ Verfügbar | `52.3` |
| `/raspclaws/cpu_usage` | ✅ Verfügbar | `18.5` |
| `/raspclaws/ram_usage` | ✅ Verfügbar | `42.1` |
| `/raspclaws/gyro_data` | ✅ **Verfügbar** | `G:1.23,0.45,-0.12 A:0.15,0.08,9.81` oder `MPU:N/A` |
| `/raspclaws/servo_positions` | ❌ Nicht verfügbar | (leer) |

### Ready Mode (HW_READY - nach erstem Bewegungsbefehl)

| Topic | Status | Wert |
|-------|--------|------|
| `/raspclaws/battery` | ✅ Verfügbar | `7.42` |
| `/raspclaws/cpu_temp` | ✅ Verfügbar | `52.3` |
| `/raspclaws/cpu_usage` | ✅ Verfügbar | `18.5` |
| `/raspclaws/ram_usage` | ✅ Verfügbar | `42.1` |
| `/raspclaws/gyro_data` | ✅ Verfügbar | `G:1.23,0.45,-0.12 A:0.15,0.08,9.81` oder `MPU:N/A` |
| `/raspclaws/servo_positions` | ✅ **Verfügbar** | `L1:266,310 L2:334,279 ...` |

## Gyro-Daten Format

### Sensor verbunden und funktionsfähig:
```bash
$ ros2 topic echo /raspclaws/gyro_data
data: 'G:1.23,0.45,-0.12 A:0.15,0.08,9.81'

# Format: "G:x,y,z A:x,y,z"
# G = Gyroscope (degrees/sec)
# A = Accelerometer (m/s²)
```

### Sensor nicht verbunden:
```bash
$ ros2 topic echo /raspclaws/gyro_data
data: 'MPU:N/A'
```

### Sensor-Fehler beim Lesen:
```bash
$ ros2 topic echo /raspclaws/gyro_data
data: 'MPU:ERROR'
```

## Testing

### 1. Vor Hardware-Initialisierung (Lazy Mode)

```bash
# Container starten (neu bauen nicht vergessen!)
docker compose -f docker-compose.ros2.yml up -d

# Status prüfen (sollte HW_LAZY zeigen)
docker exec -it raspclaws_ros2 bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /raspclaws/status --once"
# Erwartete Ausgabe: data: 'HW_LAZY - ...'

# Gyro-Daten sofort abrufbar!
docker exec -it raspclaws_ros2 bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /raspclaws/gyro_data --once"
# Erwartete Ausgabe: data: 'G:...' oder 'MPU:N/A'
```

### 2. Debug-Output ansehen

```bash
# Mit Debug-Level (falls aktiviert):
docker compose -f docker-compose.ros2.yml logs -f | grep "gyro_data"

# Erwartete Zeilen:
# [DEBUG] [raspclaws_node]: Published gyro_data: G:1.23,0.45,-0.12 A:0.15,0.08,9.81
# oder:
# [DEBUG] [raspclaws_node]: Published gyro_data: MPU:N/A
```

### 3. Kontinuierlich überwachen

```bash
# Gyro-Daten alle 2 Sekunden (entspricht Timer-Intervall):
docker exec -it raspclaws_ros2 bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /raspclaws/gyro_data"

# Ctrl+C zum Beenden
```

### 4. Publish-Rate prüfen

```bash
# Sollte ~0.5 Hz sein (alle 2 Sekunden):
docker exec -it raspclaws_ros2 bash -c "source /opt/ros/humble/setup.bash && ros2 topic hz /raspclaws/gyro_data"

# Erwartete Ausgabe:
# average rate: 0.500
#         min: 2.000s max: 2.000s std dev: 0.00000s window: 10
```

## Debug-Script

Ein Debug-Script (`debug_rosserver_topics.sh`) wurde erstellt:

```bash
# Auf Pi kopieren:
scp debug_rosserver_topics.sh pi@raspberrypi:/home/pi/adeept_raspclaws/

# Im Container ausführen:
docker exec -it raspclaws_ros2 bash /ros2_ws/debug_rosserver_topics.sh
```

Das Script prüft:
- Topic-Liste
- Topic-Info
- Topic-Type
- Topic-Daten (mit Timeout)
- Node-Info
- Publish-Rate

## Mögliche Probleme & Lösungen

### Problem: Topic zeigt immer `MPU:N/A`

**Ursache**: MPU6050 Sensor nicht verbunden oder nicht erkannt

**Lösung**:
```bash
# I2C Geräte prüfen:
docker exec -it raspclaws_ros2 i2cdetect -y 1

# Erwartete Ausgabe sollte 0x68 zeigen (MPU6050 Adresse)
#      0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
# 00:          -- -- -- -- -- -- -- -- -- -- -- -- --
# ...
# 60: -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- --
```

Wenn `0x68` nicht sichtbar:
- Sensor nicht angeschlossen
- I2C-Kabel lose
- Sensor defekt

### Problem: Topic zeigt `MPU:ERROR`

**Ursache**: Sensor ist verbunden, aber Lese-Fehler

**Lösung**:
```bash
# Logs prüfen:
docker compose -f docker-compose.ros2.yml logs -f | grep "MPU6050"

# Python-Test im Container:
docker exec -it raspclaws_ros2 python3 -c "
from mpu6050 import mpu6050
sensor = mpu6050(0x68)
print(sensor.get_accel_data())
"
```

### Problem: Topic ist leer (kein Echo)

**Ursache**: ROS2 DDS Problem oder Publisher nicht gestartet

**Lösung**:
```bash
# 1. Topic existiert?
ros2 topic list | grep gyro_data

# 2. Node läuft?
ros2 node list | grep raspclaws

# 3. Publisher aktiv?
ros2 topic info /raspclaws/gyro_data

# Sollte zeigen:
# Publisher count: 1
```

## Betroffene Dateien

```
Server/ROSServer.py                                         - Gyro-Publish-Logik geändert
Docu/Changes/FT-ROS2-3 - ROSServer Status Integration_de.md - Doku aktualisiert
Docu/Changes/FT-ROS2-3.1 - Gyro Bugfix_de.md               - Bugfix-Doku (diese Datei)
debug_rosserver_topics.sh                                   - Debug-Script erstellt
```

## Vorteile

✅ **Konsistenz**: Gyro-Daten verhalten sich wie CPU/Battery-Daten  
✅ **Sofort verfügbar**: Kein Warten auf Hardware-Initialisierung  
✅ **Logisch**: MPU6050 ist unabhängig von Servos  
✅ **Robust**: Error Handling verhindert Crashes  
✅ **Debuggbar**: Logging hilft bei Fehlersuche  
✅ **Dokumentiert**: Klares Verhalten in Doku beschrieben

## Zusammenfassung

Der Bugfix macht Gyro-Daten **sofort verfügbar** (wie CPU, Battery, RAM), weil der MPU6050 Sensor unabhängig von den Servos ist. Der einzige Grund, warum vorher eine Hardware-Initialisierung nötig war, war ein Implementierungsfehler - nicht eine technische Einschränkung.

**Vorher**: Gyro-Daten erst nach Bewegungsbefehl → ❌ Verwirrend  
**Nachher**: Gyro-Daten sofort verfügbar → ✅ Intuitiv
