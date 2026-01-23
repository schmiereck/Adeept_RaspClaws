# ROS 2 Test Client - Quick Reference

## Übersicht

Einfacher Python-basierter Test-Client für RaspClaws ROS 2 API.

**Keine Installation von Cosmos Reason 2 oder ROS Planner nötig!**

---

## Voraussetzungen

- ROS 2 Humble installiert
- Python 3 mit rclpy

```bash
source /opt/ros/humble/setup.bash
pip3 install rclpy  # Falls noch nicht installiert
```

---

## Verwendung

### Interaktiver Modus (empfohlen)

```bash
python3 ros2_test_client.py
```

Dann Befehle eingeben:
```
raspclaws> forward
raspclaws> status
raspclaws> head_left
raspclaws> stop
raspclaws> quit
```

### Einzelne Befehle

```bash
# Bewegung
python3 ros2_test_client.py forward
python3 ros2_test_client.py forward --speed 0.3
python3 ros2_test_client.py left
python3 ros2_test_client.py stop

# Kopf
python3 ros2_test_client.py head_left
python3 ros2_test_client.py head_up
python3 ros2_test_client.py head_center

# Services
python3 ros2_test_client.py reset
python3 ros2_test_client.py smooth_on

# Monitoring
python3 ros2_test_client.py status
python3 ros2_test_client.py battery
python3 ros2_test_client.py monitor --duration 30
```

### Hilfe

```bash
python3 ros2_test_client.py list
```

---

## Befehls-Referenz

| Kategorie | Befehl | Beschreibung |
|-----------|--------|--------------|
| **Bewegung** | `forward [--speed 0.5]` | Vorwärts fahren |
| | `backward [--speed 0.5]` | Rückwärts fahren |
| | `left [--speed 0.5]` | Links drehen |
| | `right [--speed 0.5]` | Rechts drehen |
| | `stop` | Stoppen |
| **Kopf** | `head_left` | Kopf nach links |
| | `head_right` | Kopf nach rechts |
| | `head_up` | Kopf nach oben |
| | `head_down` | Kopf nach unten |
| | `head_center` | Kopf zentrieren |
| **Services** | `reset` | Servos auf Home-Position |
| | `smooth_on` | Smooth Movement aktivieren |
| | `smooth_off` | Smooth Movement deaktivieren |
| | `smooth_cam_on` | Smooth Camera aktivieren |
| | `smooth_cam_off` | Smooth Camera deaktivieren |
| **Monitoring** | `status` | Status anzeigen |
| | `battery` | Batterie anzeigen |
| | `monitor [--duration 10]` | Alle Topics überwachen |

---

## Beispiele

### Einfacher Test

```bash
# 1. Status prüfen
python3 ros2_test_client.py status

# 2. Vorwärts fahren
python3 ros2_test_client.py forward

# 3. Nach 2 Sekunden stoppen
sleep 2
python3 ros2_test_client.py stop
```

### Vollständiger Test-Ablauf

```bash
# Interaktiver Modus
python3 ros2_test_client.py

# Im interaktiven Modus:
raspclaws> status
raspclaws> reset
raspclaws> forward
raspclaws> stop
raspclaws> head_left
raspclaws> head_center
raspclaws> smooth_on
raspclaws> battery
raspclaws> quit
```

### Monitoring

```bash
# 30 Sekunden alle Topics überwachen
python3 ros2_test_client.py monitor --duration 30
```

---

## Troubleshooting

**Problem**: `ROS 2 not available`
```bash
source /opt/ros/humble/setup.bash
pip3 install rclpy
```

**Problem**: `Node not found`
```bash
# ROS_DOMAIN_ID prüfen
echo $ROS_DOMAIN_ID

# Muss gleich sein wie auf Pi (Standard: 0)
export ROS_DOMAIN_ID=0
```

**Problem**: `No response from robot`
- Ist der Pi erreichbar? `ping 192.168.2.126`
- Läuft der ROS 2 Server auf dem Pi?
- Firewall blockiert? `sudo ufw allow from 192.168.2.0/24`

---

## Vorteile

✅ **Einfach**: Nur Python und ROS 2 nötig  
✅ **Schnell**: Keine Installation von Cosmos/Planner  
✅ **Interaktiv**: Live-Befehle während der Entwicklung  
✅ **Vollständig**: Alle ROS 2 API Funktionen verfügbar  
✅ **Cross-Platform**: Windows, Linux, Mac  

---

## Nächste Schritte

1. Einfache Tests mit Test-Client
2. Dann Cosmos Reason 2 installieren
3. Dann ROS Planner hinzufügen

**Vollständige Dokumentation**: `Docu/ROS2_INTEGRATION_DE.md`

---

**Branch**: ros2  
**Datei**: `ros2_test_client.py`  
**Status**: ✅ Produktionsbereit
