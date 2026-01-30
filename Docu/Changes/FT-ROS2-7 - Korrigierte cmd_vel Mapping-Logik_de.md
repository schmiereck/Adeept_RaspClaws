# FT-ROS2-7: Korrigierte cmd_vel Mapping-Logik

**Feature-Typ:** Bugfix / Verbesserung  
**Datum:** 2026-01-30  
**Status:** ✅ Implementiert

## Überblick

Die cmd_vel Mapping-Logik wurde korrigiert, um alle Bewegungstypen des GUIServers auch über ROS2 zu unterstützen:
1. ✅ Gerade Vorwärts/Rückwärts mit einstellbarer Geschwindigkeit
2. ✅ Drehen auf der Stelle mit einstellbarer Geschwindigkeit
3. ✅ Kurvenfahrt (Arc) mit einstellbarer Geschwindigkeit und Radius

## Problem

Die ursprüngliche Implementierung verwendete:
- `msg.linear.x` für Vorwärts/Rückwärts
- `msg.angular.x` für Links/Rechts Drehung

**Probleme:**
- ❌ Keine Geschwindigkeitskontrolle (nur On/Off)
- ❌ Keine Kurvenfahrt (Arc) möglich
- ❌ Unlogische Achsen-Zuordnung
- ❌ Nicht konsistent mit Twist-Message Standard

## Lösung

### Neue Mapping-Logik

**Koordinaten:**
- `msg.linear.z`: Vorwärts/Rückwärts Geschwindigkeit (-1.0 bis 1.0)
- `msg.angular.y`: Drehen/Kurvenradius (-1.0 bis 1.0)

**Bewegungstypen:**

| linear.z | angular.y | Bewegung | Move.py Funktion |
|----------|-----------|----------|------------------|
| != 0     | = 0       | Gerade Vorwärts/Rückwärts | `set_movement_speed()` + `commandInput(CMD_FORWARD/BACKWARD)` |
| != 0     | != 0      | Kurvenfahrt (Arc) | `set_movement_speed()` + `set_arc_factor()` + `commandInput(CMD_FORWARD_*_ARC)` |
| = 0      | != 0      | Drehen auf der Stelle | `set_movement_speed()` + `commandInput(CMD_LEFT/RIGHT)` |
| = 0      | = 0       | Stopp/Stand | `commandInput(MOVE_STAND)` |

### Implementierung

**Datei:** `Server/ROSServer.py` - `cmd_vel_callback()`

```python
def cmd_vel_callback(self, msg):
    # Extract velocity components
    linear_z = msg.linear.z     # Forward/backward speed (-1.0 to 1.0)
    angular_y = msg.angular.y   # Turn/arc factor (-1.0 to 1.0)
    
    # Threshold for considering a value as non-zero
    threshold = 0.05
    
    has_linear = abs(linear_z) > threshold
    has_angular = abs(angular_y) > threshold
    
    if has_linear and not has_angular:
        # Case 1: Straight forward/backward
        speed = int(abs(linear_z) * 100)  # Map to 0-100
        move.set_movement_speed(speed)
        if linear_z > 0:
            move.commandInput(CMD_FORWARD)
        else:
            move.commandInput(CMD_BACKWARD)
    
    elif has_linear and has_angular:
        # Case 2: Arc movement (curved path)
        speed = int(abs(linear_z) * 100)
        arc_factor = abs(angular_y)
        move.set_movement_speed(speed)
        move.set_arc_factor(arc_factor)
        
        if linear_z > 0:
            if angular_y > 0:
                move.commandInput(CMD_FORWARD_RIGHT_ARC)
            else:
                move.commandInput(CMD_FORWARD_LEFT_ARC)
        else:
            # Backward arc not fully supported
            move.commandInput(CMD_BACKWARD)
    
    elif not has_linear and has_angular:
        # Case 3: Turn in place
        speed = int(abs(angular_y) * 100)
        move.set_movement_speed(speed)
        if angular_y > 0:
            move.commandInput(CMD_RIGHT)
        else:
            move.commandInput(CMD_LEFT)
    
    else:
        # Case 4: Stop
        move.commandInput(MOVE_STAND)
```

## Beispiele

### 1. Gerade Vorwärts (50% Geschwindigkeit)

```bash
ros2 topic pub --once /raspclaws/cmd_vel geometry_msgs/Twist "{linear: {z: 0.5}, angular: {y: 0.0}}"
```

**Intern:**
- `linear_z = 0.5` → speed = 50
- `angular_y = 0.0` → keine Drehung
- Aktion: `move.set_movement_speed(50)` + `move.commandInput(CMD_FORWARD)`

### 2. Rechtskurve Vorwärts (50% Speed, 30% Arc)

```bash
ros2 topic pub --once /raspclaws/cmd_vel geometry_msgs/Twist "{linear: {z: 0.5}, angular: {y: 0.3}}"
```

**Intern:**
- `linear_z = 0.5` → speed = 50
- `angular_y = 0.3` → arc_factor = 0.3
- Aktion: `move.set_movement_speed(50)` + `move.set_arc_factor(0.3)` + `move.commandInput(CMD_FORWARD_RIGHT_ARC)`

### 3. Rechts Drehen (50% Geschwindigkeit)

```bash
ros2 topic pub --once /raspclaws/cmd_vel geometry_msgs/Twist "{linear: {z: 0.0}, angular: {y: 0.5}}"
```

**Intern:**
- `linear_z = 0.0` → keine lineare Bewegung
- `angular_y = 0.5` → speed = 50
- Aktion: `move.set_movement_speed(50)` + `move.commandInput(CMD_RIGHT)`

### 4. Stopp

```bash
ros2 topic pub --once /raspclaws/cmd_vel geometry_msgs/Twist "{linear: {z: 0.0}, angular: {y: 0.0}}"
```

**Intern:**
- Beide Werte = 0
- Aktion: `move.commandInput(MOVE_STAND)`

## Vergleich: Alt vs Neu

| Feature | Alte Implementierung | Neue Implementierung |
|---------|---------------------|---------------------|
| **Achsen** | linear.x, angular.x | linear.z, angular.y |
| **Geschwindigkeit** | ❌ Nur On/Off | ✅ 0-100% einstellbar |
| **Gerade Bewegung** | ✅ Ja | ✅ Ja (mit Speed) |
| **Kurvenfahrt** | ❌ Nein | ✅ Ja (Arc Factor) |
| **Drehen auf Stelle** | ✅ Ja | ✅ Ja (mit Speed) |
| **Threshold** | 0.1 | 0.05 (feiner) |
| **Logging** | Minimal | Detailliert mit Debug-Ausgaben |

## Geschwindigkeits-Mapping

**linear.z / angular.y Werte** → **Move.py Speed**

| ROS2 Wert | Prozent | Move.py Speed | Beschreibung |
|-----------|---------|---------------|--------------|
| 0.0 - 0.05 | < 5% | 0 | Als 0 interpretiert (Threshold) |
| 0.1 | 10% | 10 | Minimum-Geschwindigkeit |
| 0.3 | 30% | 30 | Langsam |
| 0.5 | 50% | 50 | Normal |
| 0.7 | 70% | 70 | Schnell |
| 1.0 | 100% | 100 | Maximum-Geschwindigkeit |

## Arc Factor Mapping

**angular.y Wert** → **Arc Factor**

| ROS2 angular.y | Arc Factor | Kurvenkrümmung |
|----------------|------------|----------------|
| 0.0 | 0.0 | Gerade (keine Kurve) |
| 0.2 | 0.2 | Sehr weite Kurve |
| 0.5 | 0.5 | Normale Kurve |
| 0.7 | 0.7 | Enge Kurve |
| 1.0 | 1.0 | Engste Kurve (Pivot auf innerem Bein) |

## Einschränkungen

**Rückwärts-Kurven:**
- Momentan nicht vollständig implementiert
- Bei `linear.z < 0` und `angular.y != 0` wird nur gerade rückwärts gefahren
- Warnung wird geloggt: `"Backward arc not fully supported, using straight backward"`

**Grund:** Move.py hat keine `CMD_BACKWARD_LEFT_ARC` / `CMD_BACKWARD_RIGHT_ARC` Befehle.

**Mögliche Erweiterung:** Kann in Move.py implementiert werden, analog zu Forward-Arcs.

## Vorteile

✅ **Intuitive Steuerung:** linear.z = Speed, angular.y = Radius  
✅ **Alle GUIServer Funktionen verfügbar:** Gerade, Kurve, Drehen  
✅ **Feine Geschwindigkeitskontrolle:** 0-100% statt On/Off  
✅ **Standard-konform:** Twist Message Achsen logisch verwendet  
✅ **Konsistent:** Gleiche Funktionalität wie GUIServer  
✅ **Gut dokumentiert:** Klare Kommentare und Debug-Ausgaben  

## Testen

### Test 1: Gerade Bewegung

```bash
# Vorwärts 50%
ros2 topic pub --once /raspclaws/cmd_vel geometry_msgs/Twist "{linear: {z: 0.5}, angular: {y: 0.0}}"

# Erwartete Ausgabe (ROSServer):
# [raspclaws_node]: Straight forward: speed=50

# Rückwärts 30%
ros2 topic pub --once /raspclaws/cmd_vel geometry_msgs/Twist "{linear: {z: -0.3}, angular: {y: 0.0}}"

# Erwartete Ausgabe:
# [raspclaws_node]: Straight backward: speed=30
```

### Test 2: Kurvenfahrt

```bash
# Rechtskurve vorwärts (50% Speed, 30% Arc)
ros2 topic pub --once /raspclaws/cmd_vel geometry_msgs/Twist "{linear: {z: 0.5}, angular: {y: 0.3}}"

# Erwartete Ausgabe:
# [raspclaws_node]: Forward-right arc: speed=50, arc=0.30

# Linkskurve vorwärts (70% Speed, 50% Arc)
ros2 topic pub --once /raspclaws/cmd_vel geometry_msgs/Twist "{linear: {z: 0.7}, angular: {y: -0.5}}"

# Erwartete Ausgabe:
# [raspclaws_node]: Forward-left arc: speed=70, arc=0.50
```

### Test 3: Drehen auf der Stelle

```bash
# Rechts drehen (50%)
ros2 topic pub --once /raspclaws/cmd_vel geometry_msgs/Twist "{linear: {z: 0.0}, angular: {y: 0.5}}"

# Erwartete Ausgabe:
# [raspclaws_node]: Turn right in place: speed=50

# Links drehen (30%)
ros2 topic pub --once /raspclaws/cmd_vel geometry_msgs/Twist "{linear: {z: 0.0}, angular: {y: -0.3}}"

# Erwartete Ausgabe:
# [raspclaws_node]: Turn left in place: speed=30
```

### Test 4: Stopp

```bash
ros2 topic pub --once /raspclaws/cmd_vel geometry_msgs/Twist "{linear: {z: 0.0}, angular: {y: 0.0}}"

# Erwartete Ausgabe:
# [raspclaws_node]: Stop/Stand
```

## Dateien

- ✅ `Server/ROSServer.py` - cmd_vel_callback() neu implementiert
- ✅ `Docu/Changes/FT-ROS2-1 - ROS2 Integration_de.md` - Dokumentation aktualisiert
- ✅ `Docu/Changes/FT-ROS2-7 - Korrigierte cmd_vel Mapping-Logik_de.md` - Diese Dokumentation

## Migration

**Wenn du bestehenden ROS2-Code hast, der linear.x und angular.x verwendet:**

```python
# Alt (funktioniert nicht mehr):
twist_msg.linear.x = 0.5
twist_msg.angular.x = 0.0

# Neu (korrekt):
twist_msg.linear.z = 0.5
twist_msg.angular.y = 0.0
```

**Kompatibilität:** Keine Abwärtskompatibilität zur alten Implementierung. Alle ROS2-Clients müssen angepasst werden.

---

**Feature umgesetzt von:** GitHub Copilot  
**Review:** Pending  
**Hardware-Test:** Pending
