#!/bin/bash
# Build-Anleitung für ROS2 Container mit libcamera Support
# Datum: 2026-01-30

echo "=========================================="
echo "ROS2 Container Build mit libcamera Support"
echo "=========================================="
echo ""
echo "✅ ÄNDERUNGEN:"
echo "  - Multi-Stage Build: Debian Bookworm → ROS2 Humble"
echo "  - libcamera + picamera2 aus Bookworm kopiert"
echo "  - Keine Host-Mounts mehr nötig"
echo ""
echo "⏱️  ERWARTETE DAUER:"
echo "  - Erster Build: ~30-60 Minuten"
echo "  - Nachfolgende Builds: ~5-10 Minuten (Cache)"
echo ""

# Prüfe ob auf Pi oder PC
if [ "$(uname -m)" != "aarch64" ]; then
    echo "⚠️  WARNUNG: Du bist nicht auf einem ARM64-System (Raspberry Pi)"
    echo "   Dieser Build funktioniert nur auf Raspberry Pi!"
    echo ""
    read -p "Trotzdem fortfahren? (y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo "=========================================="
echo "SCHRITT 1: Alte Container stoppen"
echo "=========================================="

docker compose -f docker-compose.ros2.yml down

echo "✓ Container gestoppt"
echo ""

echo "=========================================="
echo "SCHRITT 2: Image bauen (dauert lange!)"
echo "=========================================="
echo ""
echo "Stage 1/2: Debian Bookworm (libcamera)..."
echo "Stage 2/2: ROS2 Humble + Copy libcamera..."
echo ""

# Build mit Progress-Anzeige
docker compose -f docker-compose.ros2.yml build --progress=plain

if [ $? -ne 0 ]; then
    echo ""
    echo "❌ BUILD FEHLGESCHLAGEN!"
    echo ""
    echo "Häufige Fehler:"
    echo "  - Kein Internet: apt-get update schlägt fehl"
    echo "  - Wenig Speicher: Raspberry Pi braucht min. 2GB RAM + Swap"
    echo "  - Docker-Version: Benötigt Docker Compose v2+"
    echo ""
    exit 1
fi

echo ""
echo "✅ BUILD ERFOLGREICH!"
echo ""

echo "=========================================="
echo "SCHRITT 3: Container starten"
echo "=========================================="

docker compose -f docker-compose.ros2.yml up -d

echo ""
echo "✓ Container gestartet"
echo ""

echo "=========================================="
echo "SCHRITT 4: Logs prüfen"
echo "=========================================="
echo ""
echo "Warte 5 Sekunden auf Container-Start..."
sleep 5

# Prüfe ob picamera2 funktioniert
echo ""
echo "Test 1: Prüfe ob picamera2 importierbar ist..."
docker exec raspclaws_camera python3 -c "from picamera2 import Picamera2; print('✅ picamera2 import works!')" 2>&1

if [ $? -eq 0 ]; then
    echo "✅ Test 1 erfolgreich!"
else
    echo "❌ Test 1 fehlgeschlagen - picamera2 nicht verfügbar"
fi

echo ""
echo "Test 2: Prüfe ob libcamera verfügbar ist..."
docker exec raspclaws_camera python3 -c "import libcamera; print('✅ libcamera import works!')" 2>&1

if [ $? -eq 0 ]; then
    echo "✅ Test 2 erfolgreich!"
else
    echo "❌ Test 2 fehlgeschlagen - libcamera nicht verfügbar"
fi

echo ""
echo "=========================================="
echo "ZUSAMMENFASSUNG"
echo "=========================================="
echo ""
echo "Container-Status:"
docker compose -f docker-compose.ros2.yml ps
echo ""

echo "Logs ansehen:"
echo "  docker compose -f docker-compose.ros2.yml logs -f"
echo ""

echo "Container stoppen:"
echo "  docker compose -f docker-compose.ros2.yml down"
echo ""

echo "=========================================="
echo "✅ BUILD ABGESCHLOSSEN!"
echo "=========================================="
