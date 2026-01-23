# 🔧 Troubleshooting: ROSServer initialisiert Servos beim Start

## Problem
Trotz Lazy Initialization werden die Servos beim Start des Containers steif.

## Ursache
Die alte Version von ROSServer.py läuft noch im Docker-Container, obwohl die neue Version committed wurde.

---

## Lösung: Kompletter Rebuild

### Schritt 1: Aktuelle Version auf dem Pi prüfen

```bash
cd /home/pi/adeept_raspclaws

# Prüfe Git-Status
git status
git log --oneline -3

# Hole neueste Version
git fetch
git pull origin master
```

### Schritt 2: Prüfe ob Lazy Initialization in der Datei ist

```bash
# Suche nach "Lazy initialization" in ROSServer.py
grep -n "Lazy initialization" Server/ROSServer.py

# Sollte zeigen:
# 97:        # Lazy initialization state
# 101:        self.get_logger().info('💤 Lazy initialization enabled...')
```

**Falls NICHT gefunden:** Git pull hat nicht funktioniert!

```bash
# Force Pull
git fetch --all
git reset --hard origin/master
```

### Schritt 3: Container KOMPLETT neu bauen

```bash
cd /home/pi/adeept_raspclaws

# 1. Container stoppen und löschen
docker-compose -f docker-compose.ros2.yml down

# 2. ALLE alten Images löschen (wichtig!)
docker rmi $(docker images | grep raspclaws_ros2 | awk '{print $3}')

# 3. Build-Cache löschen
docker builder prune -af

# 4. NEU bauen (dauert ~10-15 Minuten)
docker-compose -f docker-compose.ros2.yml build --no-cache --pull

# 5. Starten
docker-compose -f docker-compose.ros2.yml up -d

# 6. Logs prüfen
docker-compose -f docker-compose.ros2.yml logs -f
```

### Schritt 4: Erfolgskontrolle

**Du solltest im Log sehen:**
```
[INFO] [raspclaws_node]: Initializing RaspClaws ROS 2 Node...
[INFO] [raspclaws_node]: 💤 Lazy initialization enabled - hardware will be initialized on first command
[INFO] [raspclaws_node]:    (Servos stay soft until first movement/head command)
[INFO] [raspclaws_node]: Publishers created
[INFO] [raspclaws_node]: Subscribers created
[INFO] [raspclaws_node]: Services created
[INFO] [raspclaws_node]: Timers created
[INFO] [raspclaws_node]: RaspClaws ROS 2 Node initialized successfully!
```

**KEIN "PCA9685 initialized in Move.py" beim Start!**

Erst wenn Du einen Bewegungsbefehl sendest:
```
[INFO] [raspclaws_node]: 🤖 Initializing robot hardware on first command...
PCA9685 initialized successfully on address 0x40
[INFO] [raspclaws_node]: ✓ Robot hardware initialized successfully
```

---

## Warum wird die alte Version verwendet?

### Problem 1: Docker Cache
Docker cached die Layers beim Build. Wenn der Code sich ändert, aber der Cache nicht invalided wird, wird die alte Version verwendet.

**Lösung:** `--no-cache` beim Build verwenden

### Problem 2: Git Pull funktioniert nicht
Falls lokale Änderungen existieren oder Git-Konflikte bestehen.

**Lösung:** `git reset --hard origin/master` (⚠️ lokale Änderungen gehen verloren!)

### Problem 3: Container Volume mount
Der Container mounted `/home/pi/adeept_raspclaws` → `/ros2_ws`. Falls die Dateien nicht aktualisiert wurden, läuft alte Version.

**Lösung:** Container-Restart reicht nicht, git pull muss VORHER erfolgen!

---

## Quick-Check: Welche Version läuft?

### Im Container prüfen:
```bash
docker exec -it raspclaws_ros2 /bin/bash

# Im Container:
grep -n "Lazy initialization" /ros2_ws/Server/ROSServer.py

# Sollte Treffer zeigen, sonst: ALTE VERSION!
```

---

## Komplette Neuinstallation (wenn alles andere fehlschlägt)

```bash
# 1. ALLES löschen
docker-compose -f docker-compose.ros2.yml down -v
docker system prune -af
docker volume prune -f

# 2. Code neu holen
cd /home/pi
rm -rf adeept_raspclaws
git clone https://github.com/schmiereck/Adeept_RaspClaws.git adeept_raspclaws
cd adeept_raspclaws

# 3. Auf master Branch
git checkout master
git pull

# 4. Komplett neu bauen
docker-compose -f docker-compose.ros2.yml build --no-cache --pull
docker-compose -f docker-compose.ros2.yml up -d
```

---

## Direkter Test (ohne Docker)

Falls Docker-Probleme weiter bestehen, teste direkt:

```bash
cd /home/pi/adeept_raspclaws

# ROS 2 muss installiert sein!
source /opt/ros/humble/setup.bash
sudo python3 Server/ROSServer.py
```

**Erwartung:**
- Beim Start: "💤 Lazy initialization enabled"
- Servos bleiben WEICH
- Erst bei Befehl: "🤖 Initializing robot hardware on first command"

Falls das funktioniert: **Docker-Cache-Problem!**

---

## Finale Prüfung

### Test 1: Servos beim Start weich?
```bash
# Roboter hat Akku angeschlossen
# Container starten
docker-compose -f docker-compose.ros2.yml up -d

# Beine anfassen - sind sie weich?
# ✅ JA: Lazy initialization funktioniert!
# ❌ NEIN: Alte Version läuft noch
```

### Test 2: Logs zeigen Lazy Init?
```bash
docker-compose -f docker-compose.ros2.yml logs | grep "Lazy initialization"

# Sollte zeigen:
# [INFO] [raspclaws_node]: 💤 Lazy initialization enabled...
```

### Test 3: Initialisierung erst bei Befehl?
```bash
# Bewegungsbefehl senden
ros2 topic pub --once /raspclaws/cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}}"

# Im Log sollte erscheinen:
# [INFO] [raspclaws_node]: 🤖 Initializing robot hardware on first command...
# PCA9685 initialized successfully on address 0x40
```

---

## Zusammenfassung

### ✅ Wenn Lazy Initialization funktioniert:
1. Servos sind beim Start WEICH
2. Log zeigt "💤 Lazy initialization enabled"
3. KEIN "PCA9685 initialized" beim Start
4. Erst bei Befehl: "🤖 Initializing robot hardware on first command"

### ❌ Wenn alte Version läuft:
1. Servos werden beim Start STEIF
2. Log zeigt "PCA9685 initialized in Move.py" sofort beim Start
3. Keine "Lazy initialization" Meldung

### 🔧 Fix:
```bash
cd /home/pi/adeept_raspclaws
git pull
docker-compose -f docker-compose.ros2.yml down
docker rmi $(docker images | grep raspclaws | awk '{print $3}')
docker-compose -f docker-compose.ros2.yml build --no-cache
docker-compose -f docker-compose.ros2.yml up -d
```

**Das sollte das Problem lösen!** 🎯
