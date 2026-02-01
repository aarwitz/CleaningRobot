#!/usr/bin/env bash
set -eo pipefail

# Source ROS environment
source /opt/ros/humble/setup.bash

# Source workspace if it exists
if [ -f "/opt/vision_ws/install/setup.bash" ]; then
    source /opt/vision_ws/install/setup.bash
    echo "[entrypoint] Workspace sourced from /opt/vision_ws"
else
    echo "[FATAL] Workspace not found at /opt/vision_ws"
    echo "[DEBUG] Contents of /opt:"
    ls -la /opt/ || true
    exit 30
fi

# Check for RealSense device
echo "[entrypoint] Checking for RealSense device..."
if ! lsusb | grep -i "RealSense" >/dev/null 2>&1; then
    echo "[FATAL] No RealSense device found on USB"
    lsusb 2>&1 || true
    exit 20
fi
CAMERA_ONLY=${CAMERA_ONLY:-false}
echo "[entrypoint] RealSense device found"

# Display configuration
echo "[entrypoint] =========================================="
echo "[entrypoint] Robot Configuration:"
echo "[entrypoint] - Camera Only Mode: ${CAMERA_ONLY}"
echo "[entrypoint] - SLAM: ${ENABLE_SLAM:-true}"
echo "[entrypoint] - YOLO: ${ENABLE_YOLO:-true}"
echo "[entrypoint] - Behavior: ${ENABLE_BEHAVIOR:-true}"
echo "[entrypoint] - Nav2: ${ENABLE_NAV2:-false}"
echo "[entrypoint] - Camera: ${CAM_W:-640}x${CAM_H:-480}"
echo "[entrypoint] - Network: ${NET_W:-640}x${NET_H:-640}"
echo "[entrypoint] - Model: ${MODEL_FILE_PATH:-/models/clothes2.onnx}"
echo "[entrypoint] - Arm Bridge: ${ENABLE_ARM:-true}"
echo "[entrypoint] - IMU Fusion: ${ENABLE_IMU:-false}"
echo "[entrypoint] =========================================="

# Launch unified robot bringup
echo "[entrypoint] Launching robot system..."
# Ensure the local workspace package is built so updated launch files are available
if [ -d "/opt/vision_ws/src/robot_bringup" ]; then
    echo "[entrypoint] Building robot_bringup package to pick up launch changes..."
    cd /opt/vision_ws || true
    # build only the bringup package to keep startup fast
    colcon build --packages-select robot_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release || echo "[entrypoint] robot_bringup build failed (continuing)"
    # re-source overlay after build
    if [ -f "/opt/vision_ws/install/setup.bash" ]; then
        source /opt/vision_ws/install/setup.bash
        echo "[entrypoint] Sourced updated workspace from /opt/vision_ws"
    fi
    cd - >/dev/null 2>&1 || true
fi

# Ensure overlay install takes precedence when ros2 launch resolves packages
export ROS_PACKAGE_PATH="/opt/vision_ws/install:${ROS_PACKAGE_PATH:-}"
exec ros2 launch robot_bringup robot_bringup.launch.py \
    camera_only:="${CAMERA_ONLY}" \
    enable_slam:="${ENABLE_SLAM:-true}" \
    enable_yolo:="${ENABLE_YOLO:-true}" \
    enable_behavior:="${ENABLE_BEHAVIOR:-true}" \
    enable_nav2:="${ENABLE_NAV2:-false}" \
    cam_w:="${CAM_W:-640}" \
    cam_h:="${CAM_H:-480}" \
    net_w:="${NET_W:-640}" \
    net_h:="${NET_H:-640}" \
    model_file_path:="${MODEL_FILE_PATH:-/models/clothes2.onnx}" \
    engine_file_path:="${ENGINE_FILE_PATH:-/models/clothes2.plan}" \
    confidence_threshold:="${CONF_TH:-0.65}" \
    nms_threshold:="${NMS_TH:-0.45}" \
    num_classes:="${NUM_CLASSES:-1}" \
    align_depth_enable:="${ALIGN_DEPTH:-true}" \
    enable_color:="${ENABLE_COLOR:-true}" \
    enable_depth:="${ENABLE_DEPTH:-true}" \
    enable_arm:="${ENABLE_ARM:-true}" \
    enable_imu:="${ENABLE_IMU:-false}"
    
