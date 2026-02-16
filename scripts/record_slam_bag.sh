#!/usr/bin/env bash
set -euo pipefail

BASENAME=${1:-slam_nvblox}
DURATION=${2:-60}
OUTDIR=${3:-/opt/vision_ws/bags}

mkdir -p "$OUTDIR"
FILENAME="$OUTDIR/${BASENAME}_$(date +%Y%m%d_%H%M%S)"

# Topics to record for SLAM + Nvblox visualization and debugging
# Includes images, camera info, SLAM status/pose/paths, TF, and nvblox outputs
TOPICS=(
  /visual_slam/image_0
  /visual_slam/image_1
  /visual_slam/camera_info_0
  /visual_slam/camera_info_1
  /camera/color/image_raw
  /camera/aligned_depth_to_color/image_raw
  /visual_slam/status
  /visual_slam/tracking/odometry
  /visual_slam/tracking/slam_path
  /nvblox_node/mesh
  /nvblox_node/tsdf_layer
  /nvblox/pointcloud
  /tf
  /tf_static
)

CMD=("ros2" "bag" "record" "-o" "$FILENAME")
for t in "${TOPICS[@]}"; do
  CMD+=("$t")
done

# Source ROS and workspace if available
if [ -f /opt/vision_ws/install/setup.bash ]; then
  # shellcheck disable=SC1091
  # work around 'set -u' scripts that reference unset env vars
  set +u
  source /opt/vision_ws/install/setup.bash
  set -u
fi

# Short pause to let operator start moving the robot
echo "Pausing 3s before recording so you can begin moving the robot..."
sleep 3

if [ -n "$DURATION" ]; then
  echo "Recording ros2 bag to $FILENAME for $DURATION seconds"
  timeout "$DURATION" "${CMD[@]}"
else
  echo "Recording ros2 bag to $FILENAME until interrupted (CTRL-C)"
  "${CMD[@]}"
fi

echo "Bag saved under $FILENAME*"
