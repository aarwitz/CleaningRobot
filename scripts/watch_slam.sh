#!/usr/bin/env bash

# Watch SLAM status with useful info
set +u
source /opt/vision_ws/install/setup.bash
set -u

echo "Monitoring SLAM status... (Ctrl+C to stop)"
echo "=================================================="
echo "MOVEMENT INSTRUCTIONS:"
echo "  1. Drive FORWARD 2 meters (straight line!)"
echo "  2. Turn 90° and drive forward 2 meters"
echo "  3. Make a square pattern"
echo "  4. Watch for vo_state to change from 1 → 2"
echo "=================================================="
echo ""

while true; do
  STATUS=$(ros2 topic echo /visual_slam/status --once 2>/dev/null)
  VO_STATE=$(echo "$STATUS" | grep "vo_state:" | awk '{print $2}')
  
  # Get pose to show movement
  ODOM=$(ros2 topic echo /visual_slam/tracking/odometry --once 2>/dev/null | grep -A 3 "position:")
  X=$(echo "$ODOM" | grep "x:" | head -1 | awk '{print $2}')
  Y=$(echo "$ODOM" | grep "y:" | head -1 | awk '{print $2}')
  
  clear
  echo "=================================================="
  echo "SLAM Status Monitor - $(date +%H:%M:%S)"
  echo "=================================================="
  
  if [ "$VO_STATE" = "1" ]; then
    echo "Status: VISUAL_ONLY (need TRACKING for mesh)"
    echo "Action: DRIVE FORWARD IN STRAIGHT LINES"
  elif [ "$VO_STATE" = "2" ]; then
    echo "Status: ✓ TRACKING! (ready for recording)"
    echo "Action: Keep moving - map is building!"
  else
    echo "Status: NOT_READY (vo_state: $VO_STATE)"
  fi
  
  echo ""
  echo "Position: X=${X}, Y=${Y}"
  echo ""
  echo "Key: vo_state: 1=VISUAL_ONLY, 2=TRACKING"
  echo "=================================================="
  
  sleep 1
done
