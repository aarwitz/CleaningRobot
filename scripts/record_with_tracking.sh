#!/usr/bin/env bash
set -euo pipefail

# Script to monitor SLAM, allow movement warmup, then record with mesh verification
# Usage: ./record_with_tracking.sh [output_dir]

OUTDIR="${1:-/tmp}"
WARMUP_TIME=30
RECORD_TIME=60
TRACKING_TIMEOUT=90

# Source ROS workspace
set +u
source /opt/vision_ws/install/setup.bash
set -u

echo "========================================================"
echo "SLAM + Nvblox Recording Script"
echo "========================================================"
echo "This script will:"
echo "  1. Wait up to ${TRACKING_TIMEOUT}s for SLAM to reach TRACKING (vo_state: 2)"
echo "  2. Give you ${WARMUP_TIME}s to move robot and build map"
echo "  3. Record for ${RECORD_TIME}s while monitoring mesh generation"
echo ""
echo "START MOVING THE ROBOT NOW (circles + rotation)"
echo "========================================================"

# Phase 1: Monitor for TRACKING
echo ""
echo "[Phase 1] Monitoring for SLAM TRACKING..."
SECONDS=0
while [ $SECONDS -lt $TRACKING_TIMEOUT ]; do
  # Check current vo_state
  VO_STATE=$(ros2 topic echo /visual_slam/status --once 2>/dev/null | grep "vo_state:" | awk '{print $2}' || echo "0")
  
  if [ "$VO_STATE" = "2" ]; then
    echo "✓ SLAM reached TRACKING mode after ${SECONDS}s!"
    break
  fi
  
  # Progress feedback every 5 seconds
  if [ $((SECONDS % 5)) -eq 0 ]; then
    echo "  [${SECONDS}s] Waiting for TRACKING (current: vo_state=${VO_STATE})..."
  fi
  
  sleep 1
  SECONDS=$((SECONDS + 1))
done

if [ "$VO_STATE" != "2" ]; then
  echo "⚠ WARNING: SLAM did not reach TRACKING within ${TRACKING_TIMEOUT}s"
  echo "⚠ Recording may not produce mesh data. Continue anyway? (y/n)"
  read -r -n 1 REPLY
  echo
  if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Aborted."
    exit 1
  fi
else
  # Phase 2: Warmup period for map building
  echo ""
  echo "[Phase 2] TRACKING active! Warmup period: ${WARMUP_TIME}s"
  echo "KEEP MOVING to build a good map (explore different areas)..."
  
  for ((i=WARMUP_TIME; i>0; i--)); do
    if [ $((i % 5)) -eq 0 ] || [ $i -le 3 ]; then
      echo "  Recording starts in ${i}s..."
    fi
    sleep 1
  done
fi

# Phase 3: Recording
echo ""
echo "[Phase 3] Starting ${RECORD_TIME}s recording..."
FILENAME="$OUTDIR/slam_nvblox_$(date +%Y%m%d_%H%M%S)"

TOPICS=(
  /visual_slam/status
  /visual_slam/tracking/odometry
  /visual_slam/tracking/slam_path
  /camera/color/image_raw
  /camera/aligned_depth_to_color/image_raw
  /nvblox_node/mesh
  /nvblox_node/tsdf_layer
  /tf
  /tf_static
)

echo "Recording to: $FILENAME"
echo "Topics: ${TOPICS[*]}"
echo "KEEP MOVING THE ROBOT!"
echo ""

# Start recording in background
ros2 bag record -o "$FILENAME" "${TOPICS[@]}" > /tmp/record.log 2>&1 &
RECORD_PID=$!

# Monitor recording progress and mesh generation
echo "Monitoring mesh generation..."
for ((i=1; i<=RECORD_TIME; i++)); do
  if [ $((i % 10)) -eq 0 ]; then
    # Check mesh topic every 10 seconds
    MESH_HZ=$(timeout 2 ros2 topic hz /nvblox_node/mesh --window 10 2>&1 | grep "average rate" | awk '{print $3}' || echo "0")
    if [ "$MESH_HZ" != "0" ] && [ -n "$MESH_HZ" ]; then
      echo "  [${i}s] ✓ Mesh publishing at ${MESH_HZ} Hz"
    else
      echo "  [${i}s] ⚠ No mesh detected yet (TSDF may still be integrating...)"
    fi
  fi
  sleep 1
done

# Stop recording
echo ""
echo "Stopping recording..."
kill -SIGINT $RECORD_PID 2>/dev/null || true
wait $RECORD_PID 2>/dev/null || true

# Phase 4: Verification
echo ""
echo "[Phase 4] Verifying recorded data..."
sleep 2

ros2 bag info "$FILENAME" > /tmp/bag_info.txt 2>&1

echo "Bag info:"
cat /tmp/bag_info.txt | grep -E "Duration|Messages|mesh|tsdf_layer"

MESH_COUNT=$(grep "/nvblox_node/mesh" /tmp/bag_info.txt | grep -o "Count: [0-9]*" | awk '{print $2}' || echo "0")
TSDF_COUNT=$(grep "/nvblox_node/tsdf_layer" /tmp/bag_info.txt | grep -o "Count: [0-9]*" | awk '{print $2}' || echo "0")

echo ""
echo "========================================================"
echo "Recording Complete!"
echo "========================================================"
echo "Location: $FILENAME"
echo "Mesh messages: $MESH_COUNT"
echo "TSDF messages: $TSDF_COUNT"

if [ "$MESH_COUNT" -gt 0 ]; then
  echo "✓ SUCCESS: Mesh data captured!"
else
  echo "⚠ WARNING: No mesh messages captured"
  echo "  This may indicate:"
  echo "  - SLAM was not in TRACKING long enough"
  echo "  - Not enough robot movement/scene features"
  echo "  - Nvblox mesh integration thresholds not met"
fi

echo "========================================================"
