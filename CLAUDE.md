# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

An autonomous clothes/sock-collecting mobile robot built on **ROS 2 Humble** running inside an **NVIDIA Isaac ROS** Docker container on a **Jetson** (aarch64, JetPack r36.4.4). The robot wanders, detects clothing with YOLOv8, localizes with stereo Visual SLAM, drives to targets, and picks them with a Waveshare RoArm v2 arm.

Everything runs in one container; there is no host-side ROS install. Develop by editing files under `src/` (bind-mounted) and restarting the container — the entrypoint rebuilds the workspace on every start.

## Build & run

```bash
# Build the image (first time, or after Dockerfile / apt-dependency changes)
docker compose -f docker/docker-compose.yml build        # add --no-cache to force

# Run the full stack (entrypoint colcon-builds bind-mounted src, then ros2 launch)
docker compose -f docker/docker-compose.yml up
```

Because `../src` is bind-mounted into the container and `scripts/entrypoint.sh` runs `colcon build --symlink-install` at startup, **editing a node under `src/` only requires restarting the container (`up`), not a full image rebuild**. Rebuild the image only when you change the `Dockerfile`, apt/pip dependencies, or models.

The running container is named `docker-vision-1`. Exec in with:

```bash
docker exec -it docker-vision-1 bash
source /opt/vision_ws/install/setup.bash
ros2 topic list
```

## Configuration (important gotcha)

The system is configured by **environment variables in `docker/docker-compose.yml`**, which `entrypoint.sh` passes as `ros2 launch` arguments. The defaults declared inside `robot_bringup.launch.py` (`DeclareLaunchArgument`) are **overridden** by the entrypoint and are frequently stale/different — do not trust them. `docker-compose.yml` env vars are the source of truth for what actually runs.

Key toggles (`ENABLE_*`): `SLAM`, `YOLO`, `NVBLOX`, `BEHAVIOR`, `NAV2`, `ARM`, `IMU`, `DEPTH`, `COLOR`, `VISUALIZATION`, plus `ALIGN_DEPTH`, `CAMERA_FPS`, YOLO thresholds (`CONF_TH`, `NMS_TH`, `NUM_CLASSES`), and model paths (`MODEL_FILE_PATH`, `ENGINE_FILE_PATH`). Nvblox requires `ENABLE_DEPTH`, `ENABLE_COLOR`, and `ALIGN_DEPTH` all true.

ROS networking is confined to localhost (`ROS_LOCALHOST_ONLY=1`, FastDDS). Cross-machine DDS was removed; visualization is local web-only.

## Architecture

Single launch file `src/robot_bringup/launch/robot_bringup.launch.py` brings up everything as proper ROS nodes/composable nodes (no background bash jobs). Pipeline:

1. **RealSense D455** (`realsense2_camera`) — stereo infrared (used for SLAM), depth, color, IMU. Infra rectification is disabled on purpose; SLAM consumes raw infra. A `TimerAction` re-applies `depth_module.enable_auto_exposure` 3 s after start (a fix for a frame-rate regression — see launch-file comment).
2. **Visual SLAM** (`isaac_ros_visual_slam`, composable) — stereo + IMU odometry from the two infra cameras. Publishes `map→odom→base_link` TF and `/visual_slam/tracking/*`.
3. **Nvblox** (`nvblox_ros`, composable, optional) — 3D TSDF/ESDF volumetric map from aligned depth + color + SLAM pose; produces ESDF 2D slice for Nav2 costmaps.
4. **YOLOv8 detection** — the launch file uses the **Isaac ROS** path: `isaac_ros_dnn_image_encoder` → `isaac_ros_tensor_rt` (TensorRT) → `isaac_ros_yolov8` decoder, all in `vision_container`, publishing `/yolo/detections` (`vision_msgs/Detection2DArray`).
5. **clothes_perception** node — relays the best 2D detection on `/clothes/detected`, and computes 3D points on demand (7×7 depth-window median + pinhole back-projection) via service. Kept lightweight; no continuous depth math.
6. **behavior_manager** node — the mission state machine: `WANDER → APPROACH_CLOTHES → PICK → GO_TO_BASKET → PLACE → (RECOVER)`. It is the **only** component that sends Nav2 goals, and it gates perception on/off per state.
7. **motor_controller** node — velocity PID over I2C (bus 7, addr `0x34`) driving `/cmd_vel`. Only launched when `ENABLE_NAV2=true`.
8. **arm_bridge** node — Waveshare RoArm v2 over serial (`/dev/ttyUSB0`, 115200). Own pick-place loop `IDLE→DETECTING→PICKING→PLACING→COOLDOWN`; converts camera-frame 3D points to arm coordinates via hand-eye calibration in `realsense_to_robot_coords()`.
9. **robot_state_publisher** — TF from `src/robot_bringup/urdf/robot.urdf.xacro`.
10. **rosbridge** (port 9090) + **http.server** (port 8080, serves the `*_viewer.html` files) for web visualization.

### ROS packages (`src/`)

- `robot_bringup` — launch files, URDF, Nav2 params, viewer HTML resources. The orchestration package.
- `behavior_manager` / `behavior_manager_interfaces` — state machine + custom service defs (`Get3DPose`, `GetSock3D`, `SetDetectionRate`).
- `clothes_perception` — 2D→3D detection bridge.
- `arm_bridge` — RoArm v2 serial control.
- `motor_controller` — I2C velocity controller.
- `yolo_trt_py` — pure-Python TensorRT YOLOv8 node that bypasses the Isaac ROS GXF/NITROS pipeline (TensorRT Python API + PyTorch CUDA tensors). Since 2026-08-16 the launch file runs TWO instances of it under `enable_wrist_yolo` (socks2 on the wrist AND head cameras, feeding the pick pipeline's scout/refine), alongside an `image_transport republish` for the wrist raw topic. The Isaac ROS path (#4 above) remains the `classic` profile's detector.

Each Python package is standard ament_python with a `console_scripts` entry point named `<pkg>_node`. Add a node by registering it in that package's `setup.py` and the launch file.

## Models

`models/` holds `*.onnx` (weights) and the `*.plan` (TensorRT engine, device-specific, regenerated from ONNX). Both are gitignored. `yolov8s.onnx` is generic COCO (80 classes); `clothes2.onnx` / `socks2.onnx` are the custom single-class detectors. The `.plan` is rebuilt from the ONNX when missing/forced; it is not portable across TensorRT versions. `socks2_py.plan` is the pure-Python node's own FP32 engine — keep it separate from the Isaac `socks2.plan`. **socks2.onnx expects BGR input** (measured 2026-08-16: BGR 0.54–0.81 vs RGB 0.05–0.67); `yolo_trt_node` runs it with `bgr_input:=true`. A future RGB-standard retrain must flip that flag.

## Tests

`tests/*.py` are standalone runtime validators (not pytest unit tests) — they run against a live system inside the container and check timing/topics, e.g.:

```bash
docker exec -it docker-vision-1 bash -c \
  "source /opt/vision_ws/install/setup.bash && python3 tests/test_slam_mode_startup.py"
```

`test_slam_mode_startup.py` checks per-imager framerate (≥30 Hz), inter-frame jitter (±2 ms), and inter-imager timestamp offset (±100 µs) on `/visual_slam/image_0|1`. The others validate publishers/detect-mode at startup.

## Visualization

Open in a browser on the same machine while the stack runs:
- SLAM: `http://localhost:8080/slam_viewer.html`
- Nvblox: `http://localhost:8080/nvblox_viewer.html`
- Detections: `http://localhost:8080/detection_viewer.html`

These talk to rosbridge at `ws://localhost:9090`.

## Hardware

RealSense D455 (USB 3.0); motor driver on I2C bus 7 addr `0x34`; Waveshare RoArm v2 on `/dev/ttyUSB0` @ 115200. The compose file runs `privileged`, `network_mode: host`, NVIDIA runtime, and maps `/dev/bus/usb`, `/dev/i2c-7`, and `/dev`.

## Operating the robot (HARD RULE)

**All robot motion goes through `scripts/robot` — never ad-hoc.** Do not
docker-exec bespoke python that commands the arm, do not publish to
`/teleop/action`, `/teleop/cmd`, or `/cmd_vel` by hand, and do not invoke
`pick_pipeline.py` / `pi_bridge.py` directly. The sanctioned modes are:

```
scripts/robot status       # health snapshot (moves nothing)
scripts/robot pick ...     # teacher pick; --wrist-detector yolo = on-device
                           # socks2 + head-scout fusion (recommended)
scripts/robot pi ...       # π0 policy episode (--execute to move)
scripts/robot calibrate    # wrist Jacobian re-measure
scripts/robot anchor ...   # self-anchor grasp-drop-observe
scripts/robot classic ...  # classic YOLO 2D→3D depth pick profile (--go arms it)
scripts/robot halt         # safe stop: kill pipelines, disarm π, lift to tuck
scripts/robot stow         # tucked safe pose
scripts/robot estop        # software E-STOP (always allowed)
```

It enforces a single-instance lock, per-mode preflight (teleop link, wrist
cam, DINO/π tunnels), and appends every invocation to `~/robot_runs.log`.
If a needed capability is missing, ADD A MODE to `scripts/robot` (small,
vetted, preflighted) rather than bypassing it. Read-only observation
(`ros2 topic echo`, grabbing camera frames) is fine outside the wrapper.
