#!/bin/bash
# Local ROS 2 Setup Test Script
# Run this locally to verify the container and ROS 2 topics are present

echo "========================================"
echo "Local ROS 2 Setup Test"
echo "========================================"
echo ""

# Check container status
echo "1. Checking container status..."
if docker ps --format '{{.Names}}' | grep -q "docker-vision-1"; then
    echo "   ✅ Container docker-vision-1 is running"
else
    echo "   ❌ Container docker-vision-1 not found"
    echo "   Start the stack: docker compose -f docker/docker-compose.yml up"
    exit 1
fi

echo ""
# Check ROS 2 topics inside the container
echo "2. Checking ROS 2 topic visibility inside container..."
TOPIC_COUNT=$(docker exec docker-vision-1 bash -lc "source /opt/vision_ws/install/setup.bash && ros2 topic list" 2>/dev/null | wc -l)

if [ "$TOPIC_COUNT" -eq 0 ]; then
    echo "   ❌ No topics visible inside container"
    echo "   Check: Is the container fully initialized? Wait ~30s and try again"
    exit 1
else
    echo "   ✅ Found $TOPIC_COUNT topics inside container"
fi

echo ""
# Check for key SLAM topics
echo "3. Checking for key topics..."
SLAM_TOPICS=(
    "/visual_slam/tracking/odometry"
    "/visual_slam/tracking/slam_path"
    "/nvblox_node/mesh"
    "/camera/color/image_raw"
)

for topic in "${SLAM_TOPICS[@]}"; do
    if docker exec docker-vision-1 bash -lc "source /opt/vision_ws/install/setup.bash && ros2 topic list" 2>/dev/null | grep -q "^${topic}$"; then
        echo "   ✅ $topic"
    else
        echo "   ❌ $topic NOT FOUND"
    fi
done

echo ""
# Check TF availability
echo "4. Checking TF transforms..."
if docker exec docker-vision-1 bash -lc "source /opt/vision_ws/install/setup.bash && ros2 topic list" 2>/dev/null | grep -q "^/tf$"; then
    echo "   ✅ /tf topic available"
else
    echo "   ⚠️  /tf topic not found"
fi

echo ""
# Check data rates
echo "5. Checking data rates (this takes ~3 seconds)..."
HZ_OUTPUT=$(docker exec docker-vision-1 bash -lc "source /opt/vision_ws/install/setup.bash && timeout 3 ros2 topic hz /camera/color/image_raw" 2>&1)
if echo "$HZ_OUTPUT" | grep -q "average rate"; then
    RATE=$(echo "$HZ_OUTPUT" | grep "average rate" | awk '{print $3}')
    echo "   ✅ Camera publishing at ~${RATE} Hz"
else
    echo "   ⚠️  Camera not publishing yet (may be initializing)"
fi

echo ""

echo "   Testing /visual_slam/tracking/odometry..."
HZ_OUTPUT=$(docker exec docker-vision-1 bash -lc "source /opt/vision_ws/install/setup.bash && timeout 3 ros2 topic hz /visual_slam/tracking/odometry" 2>&1)
if echo "$HZ_OUTPUT" | grep -q "average rate"; then
    RATE=$(echo "$HZ_OUTPUT" | grep "average rate" | awk '{print $3}')
    echo "   ✅ SLAM odometry publishing at ~${RATE} Hz"
else
    echo "   ⚠️  SLAM not tracking yet"
    echo "      This is normal - move robot for 30 seconds to initialize"
fi

echo ""
# Summary
echo "========================================"
echo "Summary"
echo "========================================"
echo ""
echo "If all checks passed, you can now:"
echo "  1. Open the viewer: http://localhost:8080/slam_viewer.html"
echo "  2. Monitor topics inside the container: docker exec -it docker-vision-1 bash && ros2 topic list"
echo ""
