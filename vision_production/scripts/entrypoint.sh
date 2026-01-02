#!/usr/bin/env bash
set -eo pipefail

# ROS setup scripts are NOT nounset-safe
export AMENT_TRACE_SETUP_FILES=""

source /opt/ros/humble/setup.bash

echo "[entrypoint] Checking for RealSense device..."
if ! lsusb | grep -i "RealSense" >/dev/null 2>&1; then
  echo "[FATAL] No RealSense device found on USB"
  echo "[DEBUG] Available USB devices:"
  lsusb 2>&1 || true
  exit 20
fi

echo "[entrypoint] RealSense device found via lsusb"
echo "[entrypoint] Checking librealsense device enumeration..."
if command -v rs-enumerate-devices >/dev/null 2>&1; then
  rs-enumerate-devices -s 2>&1 || echo "[WARN] rs-enumerate-devices failed"
else
  echo "[INFO] rs-enumerate-devices not available, skipping detailed check"
fi

echo "[entrypoint] Launching RealSense..."

# ------------------------------
# 1. Start RealSense driver
# ------------------------------
# SLAM requires infrared - use lower resolution for GPU optimization
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true \
  enable_depth:=true \
  enable_infra1:=true \
  enable_infra2:=true \
  enable_gyro:=true \
  enable_accel:=true \
  align_depth.enable:=false \
  enable_sync:=false \
  color_width:=640 \
  color_height:=480 \
  color_fps:=30 \
  infra_width:=640 \
  infra_height:=360 \
  infra_fps:=30 \
  depth_module.profile:=640x480x15 \
  depth_module.depth_fps:=15 \
  gyro_fps:=200 \
  accel_fps:=100 \
  unite_imu_method:=1 &
REALSENSE_PID=$!

# Wait for RealSense to publish topics
echo "[entrypoint] Waiting for camera topics..."
TIMEOUT=15
CAMERA_READY=false
deadline=$(( $(date +%s) + TIMEOUT ))
while [ "$(date +%s)" -lt "${deadline}" ]; do
  if ros2 topic list 2>/dev/null | grep -q "/camera/color/image_raw"; then
    echo "[entrypoint] Camera topics detected"
    CAMERA_READY=true
    break
  fi
  if ! kill -0 ${REALSENSE_PID} 2>/dev/null; then
    echo "[FATAL] RealSense process died unexpectedly"
    exit 22
  fi
  sleep 0.5
done

if [ "$CAMERA_READY" = false ]; then
  echo "[FATAL] Camera failed to start within ${TIMEOUT}s"
  echo "[DEBUG] RealSense process status:"
  kill -0 ${REALSENSE_PID} 2>&1 && echo "Process still running" || echo "Process died"
  echo "[DEBUG] Available topics:"
  ros2 topic list 2>&1 || true
  exit 21
fi

echo "[entrypoint] SLAM-only mode - YOLOv8 disabled for optimal navigation performance"

# ------------------------------
# 2. Vision graph disabled (prioritize SLAM for navigation)
# ------------------------------
# Uncomment to enable YOLOv8 detection:
# ros2 launch /opt/vision/vision_bringup.launch.py \
#   cam_w:="${CAM_W:-640}" cam_h:="${CAM_H:-480}" \
#   net_w:="${NET_W:-640}" net_h:="${NET_H:-640}" \
#   model_file_path:="${MODEL_FILE_PATH:-/models/yolov8s.onnx}" \
#   engine_file_path:="${ENGINE_FILE_PATH:-/models/yolov8s.plan}" \
#   confidence_threshold:="${CONF_TH:-0.7}" \
#   nms_threshold:="${NMS_TH:-0.45}" &
# VISION_PID=$!

# Give time for topics to stabilize
echo "[entrypoint] Waiting for topics to stabilize..."
sleep 5

# ------------------------------
# 3. Python scripts disabled (3D detection not needed for SLAM/Nav2)
# ------------------------------
echo "[entrypoint] 3D detection scripts disabled - running SLAM + 2D YOLOv8 only"

# ------------------------------
# 4. Clean shutdown
# ------------------------------
trap "kill ${REALSENSE_PID} 2>/dev/null || true" EXIT

# ------------------------------
# 5. Validation
# ------------------------------
/opt/vision/scripts/validate_graph.sh

# Keep container alive
tail -f /dev/null
