#!/usr/bin/env bash
set -euo pipefail

# Simple script to test bag playback and monitor nvblox mesh generation

BAG_PATH="${1:-/tmp/slam_nvblox_20260207_222616}"

echo "========================================="
echo "Testing bag playback: $BAG_PATH"
echo "========================================="

# Source ROS workspace (temporarily disable unbound variable check for sourcing)
set +u
source /opt/vision_ws/install/setup.bash
set -u

# Check initial nvblox status
echo ""
echo "Step 1: Checking nvblox mesh topic before playback..."
timeout 2 ros2 topic hz /nvblox_node/mesh -w 1 || echo "  → No mesh publishing (as expected)"

echo ""
echo "Step 2: Playing bag for 30 seconds with --clock..."
ros2 bag play "$BAG_PATH" --clock --duration 30 > /tmp/bag_play.log 2>&1 &
BAG_PID=$!
echo "  → Bag playback started (PID: $BAG_PID)"

# Wait a bit for bag to start
sleep 2

echo ""
echo "Step 3: Monitoring topics during playback..."
echo "  Checking /visual_slam/status for vo_state..."
timeout 5 ros2 topic echo /visual_slam/status --once | grep -E "vo_state|tracking_status" || echo "  → Could not get SLAM status"

echo "  Checking /nvblox_node/mesh for activity (15s)..."
timeout 16 ros2 topic hz /nvblox_node/mesh -w 10 || echo "  → No mesh messages observed during bag playback"

echo ""
echo "Step 4: Stopping bag playback..."
kill $BAG_PID 2>/dev/null || true
wait $BAG_PID 2>/dev/null || true
echo "  → Bag playback stopped"

echo ""
echo "Step 5: Checking TSDF layer updates..."
echo "  Last TSDF layer message:"
timeout 2 ros2 topic echo /nvblox_node/tsdf_layer --once | head -20 || echo "  → No TSDF layer messages"

echo ""
echo "========================================="
echo "Playback test complete. Summary:"
echo "  - Check /tmp/bag_play.log for bag playback details"
echo "  - If no mesh: likely need SLAM in TRACKING mode longer"
echo "========================================="
