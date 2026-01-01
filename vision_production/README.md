So far commands:
script session_log.txt

docker build -t vision_production:1.0 -f docker/Dockerfile .

or full rebuild

docker build --no-cache -t vision_production:1.0 -f docker/Dockerfile .


sha256sum docker/Dockerfile launch/vision_bringup.launch.py \
  scripts/*.sh models/* > checksums.txt


docker compose -f docker/docker-compose.yml up


exit

ansi2txt < session_log.txt > cleaned_log.txt


# Vision Production (Isaac ROS YOLOv8)

**Production-grade object detection system for RealSense D455 cameras.**

This system provides reliable, deterministic YOLOv8 inference with all dependencies frozen and validated.

---

## Architecture

**Optimized Pipeline (Zero-Copy NITROS)**:
```
RealSense D455 (RGB8 1280x720 @ 30Hz)
  ↓
DNN Image Encoder (resize + normalize)
  ↓
TensorRT (YOLOv8s)
  ↓
YOLOv8 Decoder
  ↓
/detections_output (vision_msgs/Detection2DArray)
```

**Key Design Decisions**:
- **No Rectify Node**: RealSense D455 outputs pre-rectified images, eliminates format conversion overhead
- **Direct RGB8 Pipeline**: Avoids BGR8↔RGB8 conversions that cause data flow issues
- **1280x720 Native Resolution**: Matches RealSense default output, no unnecessary upscaling
- **NITROS Zero-Copy**: GPU-accelerated data transfer between nodes

## Published Outputs
- `/camera/color/image_raw` - Raw camera feed (1280x720 RGB8)
- `/camera/color/camera_info` - Camera calibration
- `/detections_output` - Object detections (vision_msgs/Detection2DArray)

## Production Features

**Determinism & Reliability**:
- ✅ Base image pinned: `isaac_ros_dev-aarch64:r36.4.4_ir3.2_frozen`
- ✅ ROS packages version-locked: `ros-humble-isaac-ros-realsense=3.2.0-1*`
- ✅ TensorRT engine pre-compiled (no runtime rebuilds)
- ✅ Explicit tensor naming (no auto-negotiation)
- ✅ Validated topic wiring with health checks
- ✅ Optimized pipeline (removed unnecessary nodes)

**Failure Handling**:
Container exits with specific codes for restart orchestration:
- **Exit 10**: Required topics not publishing (camera/detections)
- **Exit 11**: Detection rate too low (< 1 Hz)

Designed for production restart-on-failure patterns.

---

## Quick Start

### Build



TODO slam + path planning + navigation pipeline:
URDF + robot_state_publisher
    publishes: base_link -> camera_link

realsense2_camera
    publishes: image, depth, imu
    publishes: camera_link -> camera_*_optical_frame

isaac_ros_visual_slam
    consumes: image, depth, imu, TF
    publishes: map -> odom (always)
    optionally publishes: odom -> base_link

isaac_ros_nvblox
    consumes: depth + TF (map → camera)

Nav2
    consumes: odom + costmap + TF
    publishes: cmd_vel

Your driver
    consumes: cmd_vel
    talks to motors over I²C