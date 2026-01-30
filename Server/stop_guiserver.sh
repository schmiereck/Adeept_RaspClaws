#!/bin/bash
# File name   : stop_guiserver.sh
# Description : Stop all running GUIServer and FPV processes
# Author      : GitHub Copilot
# Date        : 2026-01-30

echo "=========================================="
echo "Stopping GUIServer and FPV processes"
echo "=========================================="

# Find and kill all GUIServer processes
GUISERVER_PIDS=$(ps aux | grep -i 'Server/GUIServer.py' | grep -v grep | awk '{print $2}')

if [ -z "$GUISERVER_PIDS" ]; then
    echo "✓ No GUIServer processes running"
else
    echo "Found GUIServer processes: $GUISERVER_PIDS"
    for PID in $GUISERVER_PIDS; do
        echo "  Killing process $PID..."
        sudo kill -9 $PID 2>/dev/null
    done
    echo "✓ All GUIServer processes stopped"
fi

# Find and kill all standalone FPV processes
FPV_PIDS=$(ps aux | grep -i 'Server/FPV.py' | grep -v grep | awk '{print $2}')

if [ -z "$FPV_PIDS" ]; then
    echo "✓ No standalone FPV processes running"
else
    echo "Found standalone FPV processes: $FPV_PIDS"
    for PID in $FPV_PIDS; do
        echo "  Killing process $PID..."
        sudo kill -9 $PID 2>/dev/null
    done
    echo "✓ All FPV processes stopped"
fi

# Wait a moment for ports to be released
sleep 1

echo ""
echo "=========================================="
echo "Cleanup complete!"
echo "You can now start GUIServer with:"
echo "  sudo python3 Server/GUIServer.py"
echo "=========================================="
