#!/usr/bin/env bash
# SLAM verification script - run inside the container

echo "=========================================="
echo "Isaac ROS Visual SLAM Verification"
echo "=========================================="
echo ""

# Check if Visual SLAM node is running
echo "1. Checking if visual_slam node is running..."
if ros2 node list 2>/dev/null | grep -q "visual_slam"; then
  echo "   ✓ visual_slam node found"
else
  echo "   ✗ visual_slam node NOT found"
  exit 1
fi

# Check if robot_state_publisher is running
echo "2. Checking robot_state_publisher..."
if ros2 node list 2>/dev/null | grep -q "robot_state_publisher"; then
  echo "   ✓ robot_state_publisher found"
else
  echo "   ✗ robot_state_publisher NOT found"
  exit 1
fi

# Check infrared camera topics
echo "3. Checking infrared camera topics..."
if ros2 topic list 2>/dev/null | grep -q "/camera/infra1/image_rect_raw"; then
  echo "   ✓ infra1 image topic found"
else
  echo "   ✗ infra1 image topic NOT found"
fi

if ros2 topic list 2>/dev/null | grep -q "/camera/infra2/image_rect_raw"; then
  echo "   ✓ infra2 image topic found"
else
  echo "   ✗ infra2 image topic NOT found"
fi

# Check IMU topic
echo "4. Checking IMU topic..."
if ros2 topic list 2>/dev/null | grep -q "/camera/imu"; then
  echo "   ✓ IMU topic found"
else
  echo "   ✗ IMU topic NOT found"
fi

# Check SLAM output topics
echo "5. Checking SLAM output topics..."
if ros2 topic list 2>/dev/null | grep -q "/visual_slam/tracking/odometry"; then
  echo "   ✓ SLAM odometry topic found"
else
  echo "   ✗ SLAM odometry topic NOT found"
fi

# Check TF frames
echo "6. Checking TF frames..."
sleep 2  # Give TF time to populate
if ros2 run tf2_ros tf2_echo map odom 2>/dev/null | grep -q "At time"; then
  echo "   ✓ map → odom transform available"
else
  echo "   ✗ map → odom transform NOT available"
fi

if ros2 run tf2_ros tf2_echo base_link camera_link 2>/dev/null | grep -q "At time"; then
  echo "   ✓ base_link → camera_link transform available"
else
  echo "   ✗ base_link → camera_link transform NOT available"
fi

echo ""
echo "=========================================="
echo "Topic Rates (5 second sample)"
echo "=========================================="

echo "Infrared 1:"
timeout 5 ros2 topic hz /camera/infra1/image_rect_raw 2>/dev/null || echo "  No data"

echo "Infrared 2:"
timeout 5 ros2 topic hz /camera/infra2/image_rect_raw 2>/dev/null || echo "  No data"

echo "IMU:"
timeout 5 ros2 topic hz /camera/imu 2>/dev/null || echo "  No data"

echo "SLAM Odometry:"
timeout 5 ros2 topic hz /visual_slam/tracking/odometry 2>/dev/null || echo "  No data"

echo ""
echo "=========================================="
echo "Verification Complete"
echo "=========================================="
