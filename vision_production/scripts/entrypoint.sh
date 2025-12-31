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
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true \
  enable_depth:=true \
  align_depth.enable:=true \
  enable_sync:=true \
  color_width:="${CAM_W:-640}" \
  color_height:="${CAM_H:-480}" \
  color_fps:=30 \
  depth_module.profile:="${CAM_W:-640}x${CAM_H:-480}x30" &
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

echo "[entrypoint] Launching vision graph..."

# ------------------------------
# 2. Start vision graph
# ------------------------------
ros2 launch /opt/vision/vision_bringup.launch.py \
  cam_w:="${CAM_W:-640}" cam_h:="${CAM_H:-480}" \
  net_w:="${NET_W:-640}" net_h:="${NET_H:-640}" \
  model_file_path:="${MODEL_FILE_PATH:-/models/yolov8s.onnx}" \
  engine_file_path:="${ENGINE_FILE_PATH:-/models/yolov8s.plan}" \
  confidence_threshold:="${CONF_TH:-0.25}" \
  nms_threshold:="${NMS_TH:-0.45}" &
VISION_PID=$!

# Give NITROS nodes time to complete negotiation
echo "[entrypoint] Waiting for NITROS negotiation to complete..."
sleep 5

# ------------------------------
# 3. Clean shutdown
# ------------------------------
trap "kill ${VISION_PID} ${REALSENSE_PID} 2>/dev/null || true" EXIT

# ------------------------------
# 4. Validation
# ------------------------------
/opt/vision/scripts/validate_graph.sh

wait "${VISION_PID}"
