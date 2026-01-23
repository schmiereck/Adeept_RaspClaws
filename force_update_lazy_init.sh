#!/bin/bash
# Complete Force Update and Rebuild Script for Lazy Initialization
# This script ensures ALL changes are applied

set -e  # Exit on error

echo "======================================"
echo "Force Update & Rebuild for Lazy Init"
echo "======================================"
echo ""

# 1. Git: Force pull latest version
echo "Step 1: Force pulling latest version from Git..."
cd /home/pi/adeept_raspclaws
git fetch --all
git reset --hard origin/master
echo "✓ Git reset to origin/master"
echo ""

# 2. Verify all three SKIP checks are in the code
echo "Step 2: Verifying lazy init code is present..."
echo ""

echo "Checking RPIservo.py..."
if grep -q "SKIP_SERVO_AUTO_INIT" Server/RPIservo.py; then
    echo "✓ RPIservo.py has SKIP_SERVO_AUTO_INIT check"
else
    echo "✗ ERROR: RPIservo.py missing SKIP_SERVO_AUTO_INIT!"
    exit 1
fi

echo "Checking Move.py PWM init..."
if grep -q "SKIP_AUTO_INIT = os.getenv" Server/Move.py; then
    echo "✓ Move.py has SKIP_AUTO_INIT check for PWM"
else
    echo "✗ ERROR: Move.py missing SKIP_AUTO_INIT for PWM!"
    exit 1
fi

echo "Checking Move.py init_all() call..."
if grep -q "Skipping init_all() at module import" Server/Move.py; then
    echo "✓ Move.py has conditional init_all() call"
else
    echo "✗ ERROR: Move.py missing conditional init_all()!"
    echo "This is the missing piece!"
    exit 1
fi

echo ""
echo "All checks passed! Code is up-to-date."
echo ""

# 3. Docker: Complete cleanup
echo "Step 3: Cleaning up Docker..."
docker-compose -f docker-compose.ros2.yml down -v
docker rmi $(docker images | grep raspclaws | awk '{print $3}') 2>/dev/null || true
docker system prune -af
echo "✓ Docker cleaned"
echo ""

# 4. Docker: Build with no cache
echo "Step 4: Building Docker image (this takes ~10-15 minutes)..."
docker-compose -f docker-compose.ros2.yml build --no-cache --pull
echo "✓ Docker image built"
echo ""

# 5. Docker: Start container
echo "Step 5: Starting container..."
docker-compose -f docker-compose.ros2.yml up -d
echo "✓ Container started"
echo ""

# 6. Wait a bit for startup
echo "Waiting 5 seconds for container to initialize..."
sleep 5
echo ""

# 7. Check logs
echo "======================================"
echo "Checking startup logs..."
echo "======================================"
echo ""

docker-compose -f docker-compose.ros2.yml logs | head -30

echo ""
echo "======================================"
echo "Verification"
echo "======================================"
echo ""

# Count SKIP messages (should be 3)
SKIP_COUNT=$(docker-compose -f docker-compose.ros2.yml logs 2>&1 | grep -c "SKIP" || true)
echo "SKIP messages found: $SKIP_COUNT (expected: 3)"

if [ "$SKIP_COUNT" -eq 3 ]; then
    echo "✓ All three SKIP messages present!"
else
    echo "✗ ERROR: Expected 3 SKIP messages, found $SKIP_COUNT"
    echo ""
    echo "Expected:"
    echo "  1. ⏸️  Servo auto-initialization SKIPPED in RPIservo.py"
    echo "  2. ⏸️  Servo auto-initialization SKIPPED (lazy mode)"
    echo "  3. ⏸️  Skipping init_all() at module import (lazy mode)"
fi

# Check for PWM init at startup (should be NONE)
PWM_COUNT=$(docker-compose -f docker-compose.ros2.yml logs 2>&1 | head -30 | grep -c "PCA9685 initialized" || true)
echo "PWM initializations at startup: $PWM_COUNT (expected: 0)"

if [ "$PWM_COUNT" -eq 0 ]; then
    echo "✓ No PWM initialization at startup!"
else
    echo "✗ ERROR: PWM was initialized at startup!"
fi

# Check for Node success
if docker-compose -f docker-compose.ros2.yml logs 2>&1 | grep -q "RaspClaws ROS 2 Node initialized successfully"; then
    echo "✓ ROS 2 Node started successfully"
else
    echo "✗ ERROR: ROS 2 Node did not start successfully"
fi

echo ""
echo "======================================"
echo "Complete!"
echo "======================================"
echo ""
echo "To follow logs in real-time:"
echo "  docker-compose -f docker-compose.ros2.yml logs -f"
echo ""
echo "To test with a movement command:"
echo "  ros2 topic pub --once /raspclaws/cmd_vel geometry_msgs/Twist \"{linear: {x: 0.5}}\""
echo ""
