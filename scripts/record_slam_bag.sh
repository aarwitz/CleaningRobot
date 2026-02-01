#!/usr/bin/env bash
set -euo pipefail

BASENAME=${1:-slam_record}
DURATION=${2:-}
OUTDIR=${3:-/opt/vision_ws/bags}

mkdir -p "$OUTDIR"
FILENAME="$OUTDIR/${BASENAME}_$(date +%Y%m%d_%H%M%S)"

# Topics to record: minimal set that isaac_ros_visual_slam subscribes to
# (rectified/relay topics expected by the visual SLAM component)
TOPICS=(
  /visual_slam/image_0
  /visual_slam/image_1
  /visual_slam/camera_info_0
  /visual_slam/camera_info_1
)

CMD=("ros2" "bag" "record" "-o" "$FILENAME")
for t in "${TOPICS[@]}"; do
  CMD+=("$t")
done

# Source ROS and workspace if available
if [ -f /opt/vision_ws/install/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/vision_ws/install/setup.bash
fi

if [ -n "$DURATION" ]; then
  echo "Recording ros2 bag to $FILENAME for $DURATION seconds"
  timeout "$DURATION" "${CMD[@]}"
else
  echo "Recording ros2 bag to $FILENAME until interrupted (CTRL-C)"
  "${CMD[@]}"
fi

echo "Bag saved under $FILENAME*"
