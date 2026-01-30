#!/bin/bash
# Start script for RaspClaws ROS 2 Server
# Usage: ./start_ros2.sh [build|start|stop|restart|logs]

set -e

COMPOSE_FILE="docker-compose.ros2.yml"

case "$1" in
    build)
        echo "Building ROS 2 Docker image..."
        docker-compose -f $COMPOSE_FILE build
        echo "✓ Build complete"
        ;;

    start)
        echo "Starting RaspClaws ROS 2 Server..."
        echo ""
        echo "Step 1: Checking for running GUIServer processes..."
        if pgrep -f "Server/GUIServer.py" > /dev/null; then
            echo "⚠️  GUIServer is running - stopping it first (camera conflict)"
            sudo bash Server/stop_guiserver.sh
            echo "Waiting 2 seconds for ports to be released..."
            sleep 2
        else
            echo "✓ No GUIServer running"
        fi
        echo ""
        echo "Step 2: Starting Docker containers..."
        docker-compose -f $COMPOSE_FILE up -d
        echo "✓ ROS 2 Server started"
        echo ""
        echo "Check logs with: ./start_ros2.sh logs"
        echo "Stop with: ./start_ros2.sh stop"
        ;;

    start-fg)
        echo "Starting RaspClaws ROS 2 Server (foreground)..."
        echo ""
        echo "Step 1: Checking for running GUIServer processes..."
        if pgrep -f "Server/GUIServer.py" > /dev/null; then
            echo "⚠️  GUIServer is running - stopping it first (camera conflict)"
            sudo bash Server/stop_guiserver.sh
            echo "Waiting 2 seconds for ports to be released..."
            sleep 2
        else
            echo "✓ No GUIServer running"
        fi
        echo ""
        echo "Step 2: Starting Docker containers (foreground)..."
        echo "Press Ctrl+C to stop"
        docker-compose -f $COMPOSE_FILE up
        ;;

    stop)
        echo "Stopping RaspClaws ROS 2 Server..."
        docker-compose -f $COMPOSE_FILE down
        echo "✓ ROS 2 Server stopped"
        ;;

    restart)
        echo "Restarting RaspClaws ROS 2 Server..."
        docker-compose -f $COMPOSE_FILE restart
        echo "✓ ROS 2 Server restarted"
        ;;

    logs)
        echo "Showing ROS 2 Server logs (Ctrl+C to exit)..."
        docker-compose -f $COMPOSE_FILE logs -f
        ;;

    status)
        echo "RaspClaws ROS 2 Server Status:"
        docker-compose -f $COMPOSE_FILE ps
        ;;

    shell)
        echo "Opening shell in ROS 2 container..."
        docker exec -it raspclaws_ros2 /bin/bash
        ;;

    test)
        echo "Testing ROS 2 connection..."
        echo ""
        echo "1. Checking if node is running..."
        docker exec raspclaws_ros2 ros2 node list || echo "❌ Node not found"
        echo ""
        echo "2. Checking topics..."
        docker exec raspclaws_ros2 ros2 topic list || echo "❌ Cannot list topics"
        echo ""
        echo "3. Checking status topic..."
        timeout 3 docker exec raspclaws_ros2 ros2 topic echo /raspclaws/status --once || echo "❌ No status messages"
        ;;

    *)
        echo "RaspClaws ROS 2 Server Control Script"
        echo ""
        echo "Usage: $0 [command]"
        echo ""
        echo "Commands:"
        echo "  build     - Build Docker image"
        echo "  start     - Start ROS 2 server (background, auto-stops GUIServer)"
        echo "  start-fg  - Start ROS 2 server (foreground, auto-stops GUIServer)"
        echo "  stop      - Stop ROS 2 server"
        echo "  restart   - Restart ROS 2 server"
        echo "  logs      - Show logs (follow mode)"
        echo "  status    - Show container status"
        echo "  shell     - Open bash shell in container"
        echo "  test      - Run basic connectivity tests"
        echo ""
        echo "Example: $0 start"
        echo ""
        echo "Note: 'start' and 'start-fg' automatically stop GUIServer to avoid camera conflicts"
        exit 1
        ;;
esac

exit 0
