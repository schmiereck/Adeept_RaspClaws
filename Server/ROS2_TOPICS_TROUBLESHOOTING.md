# ROS 2 Topics nicht sichtbar - Troubleshooting Guide

## Problem
`ros2 topic list` zeigt keine Topics, obwohl der ROSServer läuft.

---

## Häufigste Ursache: ROS_DISCOVERY_SERVER falsch gesetzt!

### Das Problem:
```yaml
environment:
  - ROS_DISCOVERY_SERVER=127.0.0.1:11811  # ← PROBLEM!
```

**Wenn ROS_DISCOVERY_SERVER gesetzt ist:**
- ROS 2 verwendet **Discovery Server Mode** statt normaler DDS Discovery
- Node versucht sich mit Discovery Server zu verbinden
- Wenn kein Discovery Server läuft → **Keine Topics sichtbar!**

### Die Lösung:
```yaml
environment:
  # ROS_DISCOVERY_SERVER auskommentieren für normale Discovery!
  #- ROS_DISCOVERY_SERVER=127.0.0.1:11811
  - ROS_DOMAIN_ID=0
  - ROS_LOCALHOST_ONLY=0
```

**Wichtig:** ROS_DISCOVERY_SERVER nur setzen, wenn Du tatsächlich einen Discovery Server laufen hast!

---

## Weitere mögliche Ursachen

### 1. ROS_DOMAIN_ID stimmt nicht überein

**Auf dem Pi (Container):**
```bash
echo $ROS_DOMAIN_ID  # Sollte 0 sein
```

**Auf dem PC:**
```bash
echo $ROS_DOMAIN_ID  # Muss auch 0 sein!

# Falls nicht gesetzt:
export ROS_DOMAIN_ID=0
```

### 2. ROS_LOCALHOST_ONLY blockiert Netzwerk

**Prüfen:**
```bash
docker exec -it raspclaws_ros2 /bin/bash
echo $ROS_LOCALHOST_ONLY  # Sollte 0 sein
```

**Falls 1:** Nur localhost-Kommunikation erlaubt → PC kann Node nicht sehen!

### 3. Firewall blockiert DDS

**ROS 2 DDS benötigt:**
- UDP Ports: 7400-7499 (Discovery)
- UDP Ports: varies (Data)

**Prüfen:**
```bash
# Auf dem Pi:
sudo ufw status
# Falls aktiv:
sudo ufw allow from 192.168.2.0/24
```

### 4. Multicast funktioniert nicht

**Test:**
```bash
# Auf dem Pi:
ros2 multicast receive

# Auf dem PC (anderes Terminal):
ros2 multicast send
```

**Falls nicht funktioniert:** Dein Netzwerk blockiert Multicast!
→ Lösung: Discovery Server verwenden (siehe unten)

### 5. Network Mode nicht 'host'

**Prüfen in docker-compose.ros2.yml:**
```yaml
network_mode: host  # ← Muss 'host' sein!
```

**Falls 'bridge':** Container hat eigene IP → DDS Discovery funktioniert nicht!

---

## Diagnose-Schritte

### Schritt 1: Node läuft?
```bash
docker-compose -f docker-compose.ros2.yml logs | grep "initialized successfully"
# Sollte zeigen: "RaspClaws ROS 2 Node initialized successfully!"
```

### Schritt 2: Topics im Container sichtbar?
```bash
docker exec -it raspclaws_ros2 /bin/bash
ros2 node list
# Sollte zeigen: /raspclaws_node

ros2 topic list
# Sollte zeigen: /raspclaws/battery, /raspclaws/status, etc.
```

**Falls JA:** Problem ist Netzwerk-Kommunikation zwischen Pi und PC!
**Falls NEIN:** Problem ist im Container selbst!

### Schritt 3: Environment Variables prüfen
```bash
docker exec -it raspclaws_ros2 /bin/bash
env | grep ROS

# Sollte zeigen:
# ROS_DOMAIN_ID=0
# ROS_LOCALHOST_ONLY=0
# KEIN ROS_DISCOVERY_SERVER (außer Du verwendest Discovery Server!)
```

### Schritt 4: Auf dem PC prüfen
```bash
# ROS_DOMAIN_ID setzen
export ROS_DOMAIN_ID=0

# Node suchen
ros2 node list

# Topics suchen
ros2 topic list

# Daemon neustarten (kann helfen)
ros2 daemon stop
ros2 daemon start
ros2 topic list
```

### Schritt 5: Network Connectivity
```bash
# Ping Pi von PC
ping 192.168.2.126

# Check ob Port 11811 erreichbar (falls Discovery Server)
nc -zv 192.168.2.126 11811
```

---

## Lösung: Normaler DDS Discovery Mode

### docker-compose.ros2.yml:
```yaml
environment:
  # KEIN ROS_DISCOVERY_SERVER!
  - ROS_DOMAIN_ID=0
  - ROS_LOCALHOST_ONLY=0
  - PYTHONUNBUFFERED=1
  - SKIP_SERVO_AUTO_INIT=1

network_mode: host  # Wichtig!
```

### Container neu starten:
```bash
docker-compose -f docker-compose.ros2.yml down
docker-compose -f docker-compose.ros2.yml up -d
```

### Auf dem PC:
```bash
export ROS_DOMAIN_ID=0
ros2 daemon stop
ros2 daemon start
ros2 topic list
```

**Sollte jetzt funktionieren!** ✅

---

## Alternative: Discovery Server verwenden

**Wann sinnvoll:**
- Multicast funktioniert nicht
- Große Netzwerke
- VLAN/Firewall-Probleme

### 1. Discovery Server auf PC starten:
```bash
export ROS_DOMAIN_ID=0
fastdds discovery -i 0 -l 192.168.2.121 -p 11811
```

### 2. docker-compose.ros2.yml anpassen:
```yaml
environment:
  - ROS_DISCOVERY_SERVER=192.168.2.121:11811  # PC IP!
  - ROS_DOMAIN_ID=0
```

### 3. Auf dem PC:
```bash
export ROS_DISCOVERY_SERVER=192.168.2.121:11811
export ROS_DOMAIN_ID=0
ros2 topic list
```

---

## Quick Fix Checkliste

### ✅ Checklist:
- [ ] ROS_DISCOVERY_SERVER auskommentiert (für normale Discovery)
- [ ] ROS_DOMAIN_ID=0 (auf Pi und PC gleich!)
- [ ] ROS_LOCALHOST_ONLY=0
- [ ] network_mode: host
- [ ] Container neugestartet
- [ ] ros2 daemon neugestartet auf PC
- [ ] Firewall erlaubt ROS 2 Traffic

### Test:
```bash
# Auf dem Pi (im Container):
ros2 node list  # Sollte /raspclaws_node zeigen

# Auf dem PC:
export ROS_DOMAIN_ID=0
ros2 node list  # Sollte /raspclaws_node zeigen
```

---

## Häufige Fehler

### ❌ "No nodes found"
**Ursache:** ROS_DOMAIN_ID stimmt nicht überein
**Lösung:** Beide auf 0 setzen

### ❌ "Node läuft im Container, aber PC sieht nichts"
**Ursache:** ROS_DISCOVERY_SERVER gesetzt, aber kein Server läuft
**Lösung:** ROS_DISCOVERY_SERVER auskommentieren

### ❌ "Topics nach Neustart verschwunden"
**Ursache:** ros2 daemon cached alte Daten
**Lösung:** `ros2 daemon stop && ros2 daemon start`

### ❌ "Nur localhost Topics sichtbar"
**Ursache:** ROS_LOCALHOST_ONLY=1
**Lösung:** ROS_LOCALHOST_ONLY=0 setzen

---

## Debug Commands

### Container Logs:
```bash
docker-compose -f docker-compose.ros2.yml logs -f
```

### ROS 2 Node Info:
```bash
ros2 node info /raspclaws_node
```

### Topic Info:
```bash
ros2 topic info /raspclaws/status
ros2 topic echo /raspclaws/status
```

### Network Traffic:
```bash
# Zeige DDS Traffic
tcpdump -i any port 7400
```

---

## Zusammenfassung

**Das häufigste Problem:** 
`ROS_DISCOVERY_SERVER` ist gesetzt, aber kein Discovery Server läuft!

**Die Lösung:**
```yaml
# docker-compose.ros2.yml
environment:
  #- ROS_DISCOVERY_SERVER=...  # Auskommentieren!
  - ROS_DOMAIN_ID=0
  - ROS_LOCALHOST_ONLY=0
```

```bash
# Container neu starten
docker-compose -f docker-compose.ros2.yml down
docker-compose -f docker-compose.ros2.yml up -d

# Auf dem PC
export ROS_DOMAIN_ID=0
ros2 daemon stop
ros2 daemon start
ros2 topic list
```

**Das sollte funktionieren!** ✅
