#!/usr/bin/env bash
set -euo pipefail

TIMEOUT=60
SECONDS=0

echo "Monitoring /visual_slam/status up to $TIMEOUT s for vo_state==2... Please start moving the robot now (circle + rotate)."
while [ "$SECONDS" -lt "$TIMEOUT" ]; do
  if ros2 topic echo /visual_slam/status --once 2>/dev/null | grep -q "vo_state: 2"; then
    echo "SLAM TRACKING after $SECONDS s"
    /opt/vision_ws/scripts/record_slam_bag.sh slam_nvblox 60 /tmp
    rc=$?
    echo "Recorder exited with $rc"
    exit 0
  fi
  sleep 1
  SECONDS=$((SECONDS+1))
done

echo "Timeout: SLAM didn't reach TRACKING within $TIMEOUT s; starting recording anyway"
/opt/vision_ws/scripts/record_slam_bag.sh slam_nvblox 60 /tmp
rc=$?
echo "Recorder exited with $rc"

echo "Recording finished. Checking for nvblox mesh messages (hz over 5s)..."
timeout 6 ros2 topic hz /nvblox_node/mesh -w 5 || echo "nvblox mesh not publishing or no data"
echo "Done."