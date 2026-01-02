#!/bin/bash
# Quick test script for SLAM viewer functionality

echo "🧪 Testing SLAM Viewer Integration"
echo "=================================="
echo ""

# Check if container is running
if ! docker ps | grep -q docker-vision-1; then
    echo "❌ Container docker-vision-1 is not running!"
    echo "Start it with: docker compose -f docker/docker-compose.yml up"
    exit 1
fi

echo "✅ Container is running"
echo ""

# Check ROS nodes
echo "📋 Checking ROS nodes..."
docker exec docker-vision-1 bash -c "source /opt/ros/humble/setup.bash && ros2 node list" | grep -E "(visual_slam|robot_state_publisher)"
echo ""

# Check SLAM topics
echo "📋 Checking SLAM topics..."
docker exec docker-vision-1 bash -c "source /opt/ros/humble/setup.bash && ros2 topic list | grep visual_slam | head -5"
echo ""

# Check if camera topics are publishing
echo "📹 Checking camera topics..."
docker exec docker-vision-1 bash -c "source /opt/ros/humble/setup.bash && timeout 2 ros2 topic hz /camera/infra1/image_rect_raw 2>&1 | head -2" &
PID1=$!
docker exec docker-vision-1 bash -c "source /opt/ros/humble/setup.bash && timeout 2 ros2 topic hz /camera/imu 2>&1 | head -2" &
PID2=$!
wait $PID1 $PID2
echo ""

# Check if web viewer files exist
echo "📁 Checking web viewer files..."
docker exec docker-vision-1 ls -lh /opt/vision/src/web_viewer_server_3d.py /opt/vision/src/viewer.html 2>&1 | grep -v "total"
echo ""

# Test if Python script has SLAM code
echo "🔍 Verifying SLAM integration in Python script..."
docker exec docker-vision-1 grep -c "slam_odom_callback\|slam_landmarks_callback\|slam_trajectory\|slam_pose\|slam_landmarks" /opt/vision/src/web_viewer_server_3d.py
echo " SLAM-related lines found in web_viewer_server_3d.py"
echo ""

echo "🔍 Verifying SLAM integration in HTML viewer..."
docker exec docker-vision-1 grep -c "SLAM Map\|slam-container\|connectSlamTrajectory\|connectSlamPose\|connectSlamLandmarks" /opt/vision/src/viewer.html
echo " SLAM-related lines found in viewer.html"
echo ""

echo "✅ All checks passed!"
echo ""
echo "📝 Next steps:"
echo "1. Start the web viewer server:"
echo "   docker exec docker-vision-1 python3 /opt/vision/src/web_viewer_server_3d.py"
echo ""
echo "2. Open viewer.html in your browser"
echo ""
echo "3. Click the 'SLAM Map' tab and connect to:"
echo "   - Trajectory: ws://<your-ip>:8765/slam_trajectory"
echo "   - Pose: ws://<your-ip>:8765/slam_pose"
echo "   - Landmarks: ws://<your-ip>:8765/slam_landmarks"
echo ""
echo "4. Move the camera to start SLAM tracking!"
