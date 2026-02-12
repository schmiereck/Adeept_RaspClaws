# ROS2 Actions für RaspClaws - Aufbau & Nutzung

## Übersicht

Dieses Dokument beschreibt die Inbetriebnahme der ROS2 Action Server für präzise Robotersteuerung.

**Implementierte Actions:**
1. **HeadPosition** - Kamera zu Zielwinkel bewegen (Pan/Tilt)
2. **LinearMove** - Geradlinige Bewegung mit Distanzmessung
3. **Rotate** - Rotation mit optionaler IMU-Unterstützung
4. **ArcMove** - Bogenbewegung mit konfigurierbarem Radius

---

## 🏗️ Build-Anleitung (auf Raspberry Pi)

### Schritt 1: ROS2 Workspace vorbereiten

```bash
# Erstelle ROS2 Workspace (falls noch nicht vorhanden)
mkdir -p /home/pi/ros2_ws/src
cd /home/pi/ros2_ws

# Kopiere Interface-Package
cp -r /home/pi/Adeept_RaspClaws/raspclaws_interfaces src/
```

### Schritt 2: Abhängigkeiten installieren

```bash
# Aktiviere ROS2 Umgebung
source /opt/ros/humble/setup.bash  # oder micromamba activate ros_env

# Installiere Python-Abhängigkeiten (falls noch nicht vorhanden)
pip install scipy pyyaml
```

### Schritt 3: Build

```bash
cd /home/pi/ros2_ws

# Baue nur das raspclaws_interfaces Package
colcon build --packages-select raspclaws_interfaces

# Source das workspace
source install/setup.bash
```

### Schritt 3.5: Typesupport-Fix anwenden (WICHTIG für robostack/micromamba!)

**Nur bei robostack/micromamba:** Die generierten Typesupport-Module fehlt die `.so` Extension.

```bash
cd /home/pi/Adeept_RaspClaws
./fix_typesupport_extensions.sh
```

**Details:** Siehe `TYPESUPPORT_FIX.md`

### Schritt 4: Verifizierung

```bash
# Prüfe, ob Interfaces verfügbar sind
ros2 interface list | grep raspclaws

# Erwartete Ausgabe:
# raspclaws_interfaces/action/ArcMove
# raspclaws_interfaces/action/HeadPosition
# raspclaws_interfaces/action/LinearMove
# raspclaws_interfaces/action/Rotate

# Zeige Action-Definition an
ros2 interface show raspclaws_interfaces/action/HeadPosition
```

---

## 📏 Kalibrierung (WICHTIG!)

Vor der ersten Nutzung **MUSS** der Roboter kalibriert werden, um präzise Bewegungen zu ermöglichen.

### Kalibrierungs-Workflow

```bash
cd /home/pi/Adeept_RaspClaws/Server

# Starte interaktives Kalibrierungs-Script
python3 calibrate_robot.py
```

**Das Script führt durch:**
1. **Linear-Kalibrierung**: Robot fährt 10 Zyklen pro Speed-Level
   - Du misst die zurückgelegte Strecke mit einem Maßband
   - Script berechnet cm/Zyklus

2. **Rotations-Kalibrierung**: Robot dreht 5 Zyklen pro Speed-Level
   - Du misst den Drehwinkel (Markierung auf dem Robot + Winkelmesser/Bodenmarkierung)
   - Script berechnet Grad/Zyklus

**Benötigte Hilfsmittel:**
- Maßband (min. 1m)
- Winkelmesser oder Bodenmarkierungen (0°, 45°, 90°, 180°, etc.)
- Klebeband für Richtungsmarkierung auf Robot

**Ergebnis:**
- `Server/calibration.yaml` wird mit gemessenen Werten aktualisiert

**Tipp:** Kalibriere auf der Oberfläche, auf der der Robot hauptsächlich läuft (Teppich vs. Hartboden).

---

## 🚀 ROSServer starten

```bash
# Mit automatischem ROS2 workspace sourcing
cd /home/pi/Adeept_RaspClaws
./start_rosserver.sh
```

**Erwartete Log-Ausgabe:**
```
Sourcing ROS2 workspace with custom interfaces...
=== Starting ROSServer ===
ROS_DOMAIN_ID: 1
...
[INFO] [raspclaws_node]: 🎯 Initializing action servers...
[INFO] [raspclaws_node]:   ✓ HeadPosition action server created
[INFO] [raspclaws_node]:   ✓ LinearMove action server created
[INFO] [raspclaws_node]:   ✓ Rotate action server created
[INFO] [raspclaws_node]:   ✓ ArcMove action server created
[INFO] [raspclaws_node]: All action servers ready!
[INFO] [raspclaws_node]: ✓ Action servers initialized successfully
```

---

## 🧪 Testing

### Test 1: HeadPosition (Kamera bewegen)

```bash
# Bewege Kamera zu Pan=45°, Tilt=30° (smooth interpolation)
ros2 action send_goal /raspclaws/head_position \
  raspclaws_interfaces/action/HeadPosition \
  "{pan_degrees: 45.0, tilt_degrees: 30.0, smooth: true}" \
  --feedback

# Zurück zur Center-Position
ros2 action send_goal /raspclaws/head_position \
  raspclaws_interfaces/action/HeadPosition \
  "{pan_degrees: 0.0, tilt_degrees: 0.0, smooth: true}" \
  --feedback
```

**Erwartetes Verhalten:**
- Kamera bewegt sich sanft (1 Sekunde) zur Zielposition
- Feedback zeigt aktuelle Position während der Bewegung
- Result zeigt finale Position

### Test 2: LinearMove (Vorwärts/Rückwärts)

```bash
# 20cm vorwärts fahren bei Speed 35
ros2 action send_goal /raspclaws/linear_move \
  raspclaws_interfaces/action/LinearMove \
  "{distance_cm: 20.0, speed: 35.0, step_size_cm: 5.0}" \
  --feedback

# WICHTIG: Mit Maßband nachmessen!
# Sollte ~20cm sein (±1cm Toleranz)

# 20cm rückwärts fahren
ros2 action send_goal /raspclaws/linear_move \
  raspclaws_interfaces/action/LinearMove \
  "{distance_cm: -20.0, speed: 35.0, step_size_cm: 5.0}" \
  --feedback
```

**Erwartetes Verhalten:**
- Robot fährt die angegebene Distanz
- Feedback alle 5cm
- Result zeigt tatsächlich gefahrene Distanz und Anzahl Schritte

**Validierung:**
- Markiere Startposition
- Messe mit Maßband die tatsächliche Distanz
- Weicht um >5% ab? → Kalibrierung wiederholen!

### Test 3: Rotate (Drehen)

```bash
# 90° nach rechts drehen mit IMU-Feedback
ros2 action send_goal /raspclaws/rotate \
  raspclaws_interfaces/action/Rotate \
  "{angle_degrees: 90.0, speed: 35.0, step_size_deg: 15.0, use_imu: true}" \
  --feedback

# 180° nach links drehen
ros2 action send_goal /raspclaws/rotate \
  raspclaws_interfaces/action/Rotate \
  "{angle_degrees: -180.0, speed: 35.0, step_size_deg: 30.0, use_imu: true}" \
  --feedback
```

**Erwartetes Verhalten:**
- Robot dreht sich um angegebenen Winkel
- Feedback zeigt Schrittwinkel + IMU-Winkel
- Result zeigt angle_rotated (Schrittzählung), imu_angle, und angle_error

**Validierung:**
- Klebe Markierung auf Robot (zeigt nach vorne)
- Markiere Bodenwinkel (0°, 90°, 180°, 270°)
- Prüfe visuell, ob Zielwinkel erreicht wurde
- Weicht um >5° ab? → Kalibrierung wiederholen!

### Test 4: ArcMove (Bogenbewegung)

```bash
# 30cm im Bogen fahren (arc_factor=0.3)
ros2 action send_goal /raspclaws/arc_move \
  raspclaws_interfaces/action/ArcMove \
  "{distance_cm: 30.0, arc_factor: 0.3, speed: 35.0, step_size_cm: 5.0}" \
  --feedback

# Enge Kurve (arc_factor=0.7)
ros2 action send_goal /raspclaws/arc_move \
  raspclaws_interfaces/action/ArcMove \
  "{distance_cm: 40.0, arc_factor: 0.7, speed: 35.0, step_size_cm: 10.0}" \
  --feedback
```

**Erwartetes Verhalten:**
- Robot fährt in Bogenlinie
- arc_factor 0.0 = gerade, 1.0 = auf der Stelle drehen
- Feedback zeigt zurückgelegte Distanz

**Validierung:**
- Visuell prüfen, ob Bogenradius plausibel ist
- Je höher arc_factor, desto enger die Kurve

---

## 🎯 Integration Tests

### Test: Quadrat fahren

```bash
# 4x: 30cm vorwärts, 90° rechts drehen
for i in {1..4}; do
  echo "Seite $i/4"
  ros2 action send_goal /raspclaws/linear_move \
    raspclaws_interfaces/action/LinearMove \
    "{distance_cm: 30.0, speed: 40.0, step_size_cm: 10.0}" \
    --feedback

  sleep 1

  ros2 action send_goal /raspclaws/rotate \
    raspclaws_interfaces/action/Rotate \
    "{angle_degrees: 90.0, speed: 35.0, step_size_deg: 30.0, use_imu: true}" \
    --feedback

  sleep 1
done
```

**Erwartung:** Robot kehrt (ungefähr) zur Startposition zurück

### Test: Komplexe Sequenz

```bash
# Kamera nach links drehen
ros2 action send_goal /raspclaws/head_position \
  raspclaws_interfaces/action/HeadPosition \
  "{pan_degrees: -50.0, tilt_degrees: 0.0, smooth: true}" &

# Gleichzeitig: Bogenbewegung rechts
sleep 0.5
ros2 action send_goal /raspclaws/arc_move \
  raspclaws_interfaces/action/ArcMove \
  "{distance_cm: 50.0, arc_factor: 0.4, speed: 40.0, step_size_cm: 10.0}" \
  --feedback
```

---

## 🐛 Troubleshooting

### Problem: "Action interfaces not available"

**Lösung:**
```bash
# 1. Prüfe, ob Package gebaut wurde
ls /home/pi/ros2_ws/install/raspclaws_interfaces

# 2. Source das workspace
source /home/pi/ros2_ws/install/setup.bash

# 3. Prüfe Interfaces
ros2 interface list | grep raspclaws

# 4. Neustart ROSServer
./start_rosserver.sh
```

### Problem: "Calibration not available"

**Lösung:**
```bash
# Prüfe, ob calibration.yaml existiert
ls /home/pi/Adeept_RaspClaws/Server/calibration.yaml

# Falls nicht: Kalibrierung durchführen
cd /home/pi/Adeept_RaspClaws/Server
python3 calibrate_robot.py
```

### Problem: Bewegung ungenau (>5% Abweichung)

**Ursachen:**
- Kalibrierung auf falscher Oberfläche (Teppich vs. Hartboden)
- Batterie schwach (niedrige Spannung → schwächere Servos)
- Gewichtsverteilung geändert (z.B. Kamera-Upgrade)

**Lösung:**
```bash
# Neu kalibrieren auf aktueller Oberfläche
python3 calibrate_robot.py

# Bei mehreren Oberflächen: Mehrere calibration.yaml Files
# z.B.: calibration_teppich.yaml, calibration_hartboden.yaml
```

### Problem: IMU-Winkel weicht stark ab

**Ursachen:**
- Gyro-Drift (normal bei Integration über Zeit)
- Vibration während Bewegung

**Lösung:**
- System nutzt **primär Schrittzählung**, IMU nur für Fehler-Feedback
- Keine Aktion nötig, solange Schrittzählung präzise ist

### Problem: Action wird sofort cancelled

**Ursachen:**
- Robot hardware nicht initialisiert
- Ungültige Parameter (Speed außerhalb 10-80, etc.)

**Lösung:**
```bash
# 1. Prüfe ROSServer Logs auf Fehler
# 2. Prüfe Parameter-Ranges
# 3. Teste mit einfachem Goal:
ros2 action send_goal /raspclaws/head_position \
  raspclaws_interfaces/action/HeadPosition \
  "{pan_degrees: 0.0, tilt_degrees: 0.0, smooth: false}" \
  --feedback
```

---

## 📊 Parameter-Referenz

### HeadPosition
- **pan_degrees**: -100.0 bis +100.0 (links bis rechts)
- **tilt_degrees**: -67.0 bis +67.0 (unten bis oben)
- **smooth**: true = 1s interpoliert, false = direkter Sprung

### LinearMove
- **distance_cm**: Positiv = vorwärts, Negativ = rückwärts
- **speed**: 10.0 - 80.0 (höher = schneller)
- **step_size_cm**: Feedback-Intervall (z.B. 5.0 = Feedback alle 5cm)

### Rotate
- **angle_degrees**: Positiv = rechts, Negativ = links
- **speed**: 10.0 - 80.0
- **step_size_deg**: Feedback-Intervall (z.B. 15.0 = Feedback alle 15°)
- **use_imu**: true = MPU6050 Gyro-Daten in Feedback/Result

### ArcMove
- **distance_cm**: Distanz entlang des Bogens
- **arc_factor**: 0.0 = gerade, 1.0 = auf Stelle drehen (0.0-1.0)
- **speed**: 10.0 - 80.0
- **step_size_cm**: Feedback-Intervall

---

## 🔧 Erweiterte Nutzung

### Action aus Python-Code aufrufen

```python
import rclpy
from rclpy.action import ActionClient
from raspclaws_interfaces.action import LinearMove

# Node erstellen
rclpy.init()
node = rclpy.create_node('my_client')

# Action Client erstellen
action_client = ActionClient(node, LinearMove, '/raspclaws/linear_move')
action_client.wait_for_server()

# Goal erstellen und senden
goal_msg = LinearMove.Goal()
goal_msg.distance_cm = 30.0
goal_msg.speed = 40.0
goal_msg.step_size_cm = 5.0

future = action_client.send_goal_async(goal_msg, feedback_callback=my_feedback_callback)
rclpy.spin_until_future_complete(node, future)

# Result auswerten
goal_handle = future.result()
result_future = goal_handle.get_result_async()
rclpy.spin_until_future_complete(node, result_future)
result = result_future.result().result

print(f"Traveled: {result.distance_traveled}cm in {result.steps_taken} steps")
```

### Action canceln

```bash
# In anderem Terminal:
ros2 action cancel_goal /raspclaws/linear_move <goal_id>

# Oder: Ctrl+C im send_goal Terminal
```

---

## 📝 Wartung & Updates

### Kalibrierung aktualisieren

```bash
# Backup alte Kalibrierung
cp Server/calibration.yaml Server/calibration_backup_$(date +%Y%m%d).yaml

# Neue Kalibrierung
python3 Server/calibrate_robot.py
```

### Kalibrierungs-Daten anschauen

```python
from calibration import RobotCalibration

cal = RobotCalibration()
cal.print_calibration_summary()

# Teste Interpolation
print(f"Speed 35.0 -> {cal.get_cm_per_step(35.0):.2f} cm/cycle")
print(f"Speed 35.0 -> {cal.get_degrees_per_step(35.0):.2f} deg/cycle")
```

---

## 🎉 Erfolgreiche Implementierung!

Wenn alle Tests erfolgreich sind:
- ✅ Alle 4 Action Server laufen
- ✅ Kalibrierung ist präzise (< 5% Fehler)
- ✅ Robot reagiert auf Goals mit korrektem Feedback
- ✅ Komplexe Sequenzen funktionieren

**Dann ist das System produktionsreif!** 🚀

---

## 📚 Weitere Ressourcen

- ROS2 Actions Tutorial: https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html
- Kalibrierungs-Best-Practices: `Server/calibrate_robot.py --help`
- Action-Definitionen: `raspclaws_interfaces/action/*.action`

Bei Fragen oder Problemen: Siehe GitHub Issues oder Dokumentation.
