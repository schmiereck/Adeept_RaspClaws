# Kamera-Test auf dem Raspberry Pi

## 1. Container neu bauen und starten

```bash
cd ~/adeept_raspclaws

# Container stoppen
docker-compose -f docker-compose.ros2.yml down

# Nur Kamera-Container neu bauen (schneller als --no-cache)
docker-compose -f docker-compose.ros2.yml build raspclaws_camera

# Container starten
docker-compose -f docker-compose.ros2.yml up -d
```

## 2. Logs prüfen (sollte KEINE Fehler zeigen)

```bash
# Letzte 30 Zeilen
docker logs raspclaws_camera --tail 30

# Sollte zeigen:
# [v4l2_camera_node-1] [INFO] [...] Starting camera
# [image_republisher] ... (keine Errors!)
```

## 3. Topics prüfen

```bash
docker exec -it raspclaws_camera bash -c \
  "source /opt/ros/humble/setup.bash && ros2 topic list"
```

**Erwartete Ausgabe:**
```
/camera_info
/image_raw
/image_raw/compressed        ✅ KORREKT
/parameter_events
/raspclaws/battery
... (alle anderen Topics)
```

**Sollte NICHT mehr erscheinen:**
```
/out/compressed              ❌ FALSCH (war der Fehler)
```

## 4. Kamera-Framerate testen

```bash
# Raw-Image
docker exec -it raspclaws_camera bash -c \
  "source /opt/ros/humble/setup.bash && ros2 topic hz /image_raw"

# Erwartung: average rate: ~30.000 Hz
```

```bash
# Compressed-Image
docker exec -it raspclaws_camera bash -c \
  "source /opt/ros/humble/setup.bash && ros2 topic hz /image_raw/compressed"

# Erwartung: average rate: ~30.000 Hz
```

## 5. Bandwidth vergleichen

```bash
# Raw (sollte ~27 MB/s sein)
docker exec -it raspclaws_camera bash -c \
  "source /opt/ros/humble/setup.bash && ros2 topic bw /image_raw"
```

```bash
# Compressed (sollte ~1-1.5 MB/s sein)
docker exec -it raspclaws_camera bash -c \
  "source /opt/ros/humble/setup.bash && ros2 topic bw /image_raw/compressed"
```

## 6. Einzelnes Bild anzeigen (Optional)

```bash
# Camera Info
docker exec -it raspclaws_camera bash -c \
  "source /opt/ros/humble/setup.bash && ros2 topic echo /camera_info --once"

# Sollte Kamera-Parameter zeigen (width: 640, height: 480)
```

---

## Erwartete Ergebnisse

| Test | Erwartung |
|------|-----------|
| Topic-Liste | `/image_raw/compressed` vorhanden, `/out/compressed` NICHT vorhanden |
| Framerate `/image_raw` | ~30 Hz |
| Framerate `/image_raw/compressed` | ~30 Hz |
| Bandwidth Raw | ~27 MB/s |
| Bandwidth Compressed | ~1-1.5 MB/s |
| Logs | Keine Fehler/Exceptions |

---

## Falls Probleme auftreten

### Problem: "does not appear to be published yet"

```bash
# Prüfe ob Nodes laufen
docker exec -it raspclaws_camera bash -c \
  "source /opt/ros/humble/setup.bash && ros2 node list"

# Sollte zeigen:
# /image_republisher
# /v4l2_camera
```

### Problem: "/out/compressed" erscheint immer noch

```bash
# Cache löschen, komplett neu bauen
docker-compose -f docker-compose.ros2.yml down
docker rmi adeept_raspclaws-raspclaws_camera
docker-compose -f docker-compose.ros2.yml build --no-cache raspclaws_camera
docker-compose -f docker-compose.ros2.yml up -d
```

### Problem: Republisher crasht

```bash
# Zeige komplette Fehlermeldung
docker logs raspclaws_camera | grep -A 20 "Exception"
```

Dann melde dich mit der Ausgabe!

---

## Erfolg! ✅

Falls alle Tests durchlaufen:

```bash
# Speichere Konfiguration
cd ~/adeept_raspclaws
git add launch/camera.launch.py
git commit -m "FT-ROS2-4: Fix image_transport republisher remappings"
git push
```
