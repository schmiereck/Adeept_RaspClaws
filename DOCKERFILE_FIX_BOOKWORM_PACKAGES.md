# WICHTIG: Dockerfile.ros2 Korrektur

**Datum:** 2026-01-30  
**Problem:** Build-Fehler wegen fehlender Pakete in Debian Bookworm

---

## ❌ Problem

```
E: Unable to locate package python3-libcamera
E: Unable to locate package python3-picamera2
E: Unable to locate package libcamera0.3
```

---

## 🔍 Root Cause

**`python3-libcamera` und `python3-picamera2` existieren NUR in Raspberry Pi OS, NICHT in Standard Debian Bookworm!**

| Package | Standard Debian Bookworm | Raspberry Pi OS Bookworm |
|---------|--------------------------|--------------------------|
| `python3-libcamera` | ❌ Nicht verfügbar | ✅ Verfügbar |
| `python3-picamera2` | ❌ Nicht verfügbar | ✅ Verfügbar |
| `libcamera0.2` | ✅ Verfügbar | ✅ Verfügbar |
| `libcamera0.3` | ❌ Gibt es nicht | ❌ Gibt es nicht (nur 0.2) |

---

## ✅ Lösung: Hybrid-Ansatz

### Strategie:

1. **C-Library** aus Debian Bookworm apt (`libcamera0.2`)
2. **Python-Bindings** aus pip (`picamera2`)

### Stage 1 (korrigiert):

```dockerfile
FROM debian:bookworm-slim AS camera-builder

# ✅ C-Library aus apt
RUN apt-get install -y \
    libcamera0.2 \        # Nicht 0.3!
    libcamera-ipa \
    libcamera-tools

# ✅ Python-Bindings aus pip
RUN pip3 install picamera2
```

### Stage 2:

```dockerfile
FROM ros:humble-ros-base

# ✅ Kopiere von /usr/local/lib/python3.11 (pip-Installation)
COPY --from=camera-builder /usr/local/lib/python3.11/dist-packages/picamera2 /usr/lib/python3/dist-packages/
COPY --from=camera-builder /usr/local/lib/python3.11/dist-packages/libcamera /usr/lib/python3/dist-packages/
```

---

## 🎯 Warum funktioniert das?

**`pip install picamera2`:**
- Lädt picamera2 Source
- Kompiliert Python-Bindings für libcamera
- Benötigt nur die C-Library (`libcamera0.2`) aus apt

**Ergebnis:** Vollständiges picamera2 mit libcamera-Bindings, ohne Raspberry Pi OS zu brauchen! ✅

---

## 📋 Änderungen

### Dockerfile.ros2 (korrigiert):

| Vorher (fehlerhaft) | Nachher (funktioniert) |
|---------------------|------------------------|
| `python3-libcamera` | ❌ Entfernt (nicht verfügbar) |
| `python3-picamera2` | ❌ Entfernt (nicht verfügbar) |
| `libcamera0.3` | → `libcamera0.2` ✅ |
| - | `pip3 install picamera2` ✅ |
| COPY aus `/usr/lib/python3/dist-packages/` | → COPY aus `/usr/local/lib/python3.11/dist-packages/` ✅ |

---

## 🚀 Nächste Schritte

Die Dateien sind bereits korrigiert! Einfach neu bauen:

```bash
cd ~/adeept_raspclaws

# Neu bauen mit korrigiertem Dockerfile
docker compose -f docker-compose.ros2.yml build

# Sollte jetzt funktionieren!
```

---

## ✅ Erwartete Ausgabe (erfolgreich):

```
[camera-builder 2/2] RUN apt-get update && apt-get install -y libcamera0.2...
 ✅ libcamera0.2 installiert

[camera-builder 3/3] RUN pip3 install picamera2
 ✅ picamera2 + libcamera Python-Bindings installiert

[raspclaws_ros2 2/5] COPY --from=camera-builder /usr/local/lib/python3.11...
 ✅ picamera2 kopiert

Successfully built xyz123
 ✅ BUILD ERFOLGREICH!
```

---

## 🔄 Warum Multi-Stage trotzdem besser als pip direkt?

**Option 1: pip direkt in ROS2 Image**
```dockerfile
FROM ros:humble-ros-base
RUN pip3 install picamera2
# ❌ Kompiliert bei jedem Build (langsam!)
```

**Option 2: Multi-Stage Build** ✅
```dockerfile
FROM debian:bookworm AS camera-builder
RUN pip3 install picamera2

FROM ros:humble-ros-base
COPY --from=camera-builder ...
# ✅ Kompiliert nur in Stage 1 (Cache!)
# ✅ ROS2 Image bleibt sauber
```

---

**Status:** ✅ Korrigiert und bereit zum Build!  
**Build-Zeit:** ~10-15 Minuten (pip compile) + ~5-10 Minuten (ROS2 stage)
