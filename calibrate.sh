#!/bin/bash
# Wrapper script to run robot calibration
# This ensures the correct Python environment and user permissions

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Check if we're running as root
if [ "$EUID" -eq 0 ]; then
    echo "❌ ERROR: Do not run this script with sudo!"
    echo "   The script needs to run as user 'pi' to access Python packages."
    echo "   Usage: ./calibrate.sh [--test]"
    exit 1
fi

# Check if gui_server is running
if systemctl is-active --quiet gui_server.service; then
    echo "⚠️  WARNING: gui_server.service is running!"
    echo "   This may interfere with calibration."
    read -p "   Stop gui_server now? (y/n): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "   Stopping gui_server..."
        sudo systemctl stop gui_server.service
    fi
fi

# Run calibration script
echo "Starting calibration..."
python3 "$SCRIPT_DIR/Server/calibrate_robot.py" "$@"
