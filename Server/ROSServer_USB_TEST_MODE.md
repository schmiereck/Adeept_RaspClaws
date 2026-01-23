# ROSServer - USB Test Mode (ohne Akku)

## Problem
Beim Start des ROSServer werden normalerweise alle Servos initialisiert und "steif". Das erfordert eine Batterie/Akku-Stromversorgung.

## Lösung: USB-Test-Modus

Mit der Umgebungsvariable `ROS_SKIP_SERVOS=1` können die Servos übersprungen werden.

---

## Direkt auf dem Pi (ohne Docker)

### ROS 2 ist installiert:
```bash
cd /home/pi/adeept_raspclaws
export ROS_SKIP_SERVOS=1
sudo -E python3 Server/ROSServer.py
```

**Wichtig:** Das `-E` bei sudo ist notwendig, damit die Umgebungsvariable weitergegeben wird!

### ROS 2 ist NICHT installiert (Mock-Mode):
```bash
cd /home/pi/adeept_raspclaws
export ROS_SKIP_SERVOS=1
sudo -E python3 Server/ROSServer.py
```

Du siehst dann:
```
WARNING: ROS 2 not available: No module named 'rclpy'
Running in MOCK MODE - ROS 2 features disabled
⚠️  SERVO INITIALIZATION SKIPPED (ROS_SKIP_SERVOS=1)
⚠️  Robot will NOT move - USB testing mode only!
```

---

## Mit Docker

### Option A: Temporär für einen Test
```bash
cd /home/pi/Adeept_RaspClaws
docker-compose -f docker-compose.ros2.yml run -e ROS_SKIP_SERVOS=1 raspclaws_ros2
```

### Option B: Dauerhaft in docker-compose.ros2.yml ändern
```yaml
environment:
  - ROS_SKIP_SERVOS=1  # Ändere 0 auf 1
```

Dann normal starten:
```bash
docker-compose -f docker-compose.ros2.yml up -d
```

---

## Was funktioniert im USB-Test-Modus?

✅ **Funktioniert:**
- ROS 2 Node startet
- Topics werden erstellt (`/raspclaws/battery`, `/raspclaws/status`, etc.)
- Services werden erstellt (`/raspclaws/reset_servos`, etc.)
- Verbindung mit PC/Jetson kann getestet werden
- Test-Client funktioniert

❌ **Funktioniert NICHT:**
- Servos bewegen sich nicht
- Roboter kann sich nicht bewegen
- LEDs funktionieren nicht
- Schalter funktionieren nicht

---

## Test-Beispiele

### 1. Node sichtbar?
```bash
# Auf dem PC (mit ROS 2 Humble):
ros2 node list
# Sollte zeigen: /raspclaws_node
```

### 2. Topics sichtbar?
```bash
ros2 topic list
# Sollte zeigen: /raspclaws/battery, /raspclaws/status, etc.
```

### 3. Status-Nachrichten empfangen?
```bash
ros2 topic echo /raspclaws/status
```

### 4. Bewegungsbefehl senden (wird ignoriert)
```bash
# Wird akzeptiert aber Servos bewegen sich nicht
ros2 topic pub /raspclaws/cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}}"
```

### 5. Test-Client verwenden
```bash
# Auf dem PC:
python3 ros2_test_client.py status
python3 ros2_test_client.py monitor --duration 10
```

---

## Zurück zum Normal-Modus

### Direkt:
```bash
unset ROS_SKIP_SERVOS
sudo python3 Server/ROSServer.py
```

### Docker:
```bash
# In docker-compose.ros2.yml zurück ändern:
# - ROS_SKIP_SERVOS=0

docker-compose -f docker-compose.ros2.yml down
docker-compose -f docker-compose.ros2.yml up -d
```

---

## Wann den USB-Test-Modus verwenden?

✅ **Gut für:**
- ROS 2 Verbindung testen ohne Akku
- Topic/Service Discovery testen
- Netzwerk-Konfiguration prüfen
- Test-Client entwickeln
- Docker-Setup verifizieren

❌ **NICHT geeignet für:**
- Bewegungs-Tests
- Servo-Kalibrierung
- Vollständige Funktionstests
- Produktiv-Betrieb

---

## Troubleshooting

### "NameError: name 'Node' is not defined"
**Problem:** ROS 2 ist nicht installiert

**Lösung:** 
1. Docker verwenden (empfohlen), oder
2. ROS 2 Humble auf dem Pi installieren (siehe FT-ros2-1 Dokumentation)

### Servos bleiben trotzdem steif
**Problem:** `ROS_SKIP_SERVOS` Variable wird nicht erkannt

**Lösung:**
```bash
# Bei sudo IMMER -E verwenden!
sudo -E python3 Server/ROSServer.py

# Oder direkt als root:
su
export ROS_SKIP_SERVOS=1
python3 Server/ROSServer.py
```

### "WARNING: Robot modules not available"
**Normal!** Das bedeutet nur, dass der Mock-Modus aktiv ist wenn ROS 2 nicht installiert ist.

---

## Zusammenfassung

**Ein Befehl für USB-Tests:**
```bash
export ROS_SKIP_SERVOS=1 && sudo -E python3 Server/ROSServer.py
```

**Wichtig:**
- ⚠️ Nur für Tests ohne Akku
- ⚠️ Roboter bewegt sich NICHT
- ⚠️ Für echte Tests: Akku anschließen und `ROS_SKIP_SERVOS=0` (oder unset)
