#!/bin/bash
# Start ROS2 with Camera on Host (Hybrid Mode)
# Datum: 2026-01-30

cd ~/adeept_raspclaws

echo "=========================================="
echo "Starting ROS2 + Camera (Hybrid Mode)"
echo "=========================================="
echo ""
echo "Architecture:"
echo "  🐳 Docker:  ROSServer.py (robot control)"
echo "  🏠 Host:    FPV.py (camera capture)"
echo "  🏠 Host:    FPV_ROS2_simple.py (ROS2 bridge)"
echo ""

# 1. Stop old processes
echo "Step 1: Stopping old processes..."
sudo bash Server/stop_guiserver.sh 2>/dev/null || true
docker compose -f docker-compose.ros2.yml down 2>/dev/null || true
pkill -f "FPV.py" 2>/dev/null || true
pkill -f "FPV_ROS2_simple.py" 2>/dev/null || true
sleep 2
echo "✓ Old processes stopped"
echo ""

# 2. Start ROSServer in Docker
echo "Step 2: Starting ROSServer (Docker)..."
docker compose -f docker-compose.ros2.yml up -d raspclaws_ros2

if [ $? -ne 0 ]; then
    echo "❌ Failed to start Docker container!"
    exit 1
fi

echo "✓ ROSServer started in Docker"
echo ""

# 3. Start FPV.py on host
echo "Step 3: Starting FPV.py (Host)..."
python3 Server/FPV.py > /tmp/fpv.log 2>&1 &
FPV_PID=$!
echo "✓ FPV.py started (PID: $FPV_PID)"
echo "  Logs: tail -f /tmp/fpv.log"
echo ""

# 4. Wait for FPV to initialize
echo "Step 4: Waiting for FPV.py to initialize camera..."
sleep 5

# Check if FPV is still running
if ! kill -0 $FPV_PID 2>/dev/null; then
    echo "❌ FPV.py crashed during startup!"
    echo "   Check logs: cat /tmp/fpv.log"
    exit 1
fi

echo "✓ FPV.py initialized"
echo ""

# 5. Start FPV_ROS2_simple.py on host
echo "Step 5: Starting FPV_ROS2_simple.py (Host)..."

# Source ROS2 environment
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    python3 Server/FPV_ROS2_simple.py > /tmp/fpv_ros2.log 2>&1 &
    FPV_ROS2_PID=$!
    echo "✓ FPV_ROS2_simple.py started (PID: $FPV_ROS2_PID)"
    echo "  Logs: tail -f /tmp/fpv_ros2.log"
else
    echo "⚠️  ROS2 not installed on host - FPV_ROS2_simple.py not started"
    echo "   Camera will work via ZMQ, but not via ROS2 topics"
    FPV_ROS2_PID=""
fi

echo ""
echo "=========================================="
echo "✅ All services started!"
echo "=========================================="
echo ""
echo "Process IDs:"
echo "  FPV.py:              $FPV_PID"
if [ -n "$FPV_ROS2_PID" ]; then
    echo "  FPV_ROS2_simple.py:  $FPV_ROS2_PID"
fi
echo "  Docker Container:    raspclaws_ros2"
echo ""
echo "Logs:"
echo "  Docker:     docker compose -f docker-compose.ros2.yml logs -f raspclaws_ros2"
echo "  FPV:        tail -f /tmp/fpv.log"
if [ -n "$FPV_ROS2_PID" ]; then
    echo "  FPV_ROS2:   tail -f /tmp/fpv_ros2.log"
fi
echo ""
echo "Test Camera:"
if [ -n "$FPV_ROS2_PID" ]; then
    echo "  ros2 topic list | grep camera"
    echo "  ros2 topic hz /raspclaws/camera/image_compressed"
fi
echo ""
echo "Stop all:"
echo "  kill $FPV_PID"
if [ -n "$FPV_ROS2_PID" ]; then
    echo "  kill $FPV_ROS2_PID"
fi
echo "  docker compose -f docker-compose.ros2.yml down"
echo ""
echo "=========================================="
