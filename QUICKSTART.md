# Quick Start Guide

## Project Structure

```
vision_production/
├── src/                    # ROS2 workspace
│   ├── robot_bringup/     # Unified launch + config + urdf
│   ├── sock_perception/   # 3D target extraction
│   ├── behavior_manager/  # State machine
│   ├── motor_controller/  # Velocity PID for I2C motors
│   └── arm_bridge/        # Waveshare RoArm v2 control
├── docker/
│   ├── Dockerfile         # Builds workspace with colcon
│   └── docker-compose.yml # Runtime configuration
├── models/                # YOLO models (clothes2.onnx, clothes2.plan)
├── scripts/
│   └── entrypoint.sh     # Single launch command
├── QUICKSTART.md         # This file
└── README.md             # Project overview
```

**Key point:** Everything is in ROS2 packages now. No standalone scripts, no duplicate launch files.

## Build and Run

```bash
cd /home/taylor/workspaces/CleaningRobot/vision_production

# Build (compiles workspace during image build)
docker compose -f docker/docker-compose.yml build

# Or force rebuild from scratch
docker compose -f docker/docker-compose.yml build --no-cache

# Run
docker compose -f docker/docker-compose.yml up
```

That's it! The entrypoint runs one command:
```bash
ros2 launch robot_bringup robot_bringup.launch.py
```

## What's Running

The unified launch file starts:
- **RealSense D455** camera (with aligned depth enabled)
- **Visual SLAM** (stereo odometry from infrared cameras)
- **YOLOv8** detection (composable nodes in GPU-optimized container)
- **Sock perception** node (3D target extraction with temporal filtering)
- **Behavior manager** (state machine orchestrating the mission)
- **Motor controller** (velocity PID for I2C motor driver on bus 7, addr 0x34)
- **Arm bridge** (Waveshare RoArm v2 serial control at /dev/ttyUSB0)
- **Robot state publisher** (TF tree from URDF)
- **Nav2** (optional, disabled by default)

## Hardware Requirements

- **RealSense D455** on USB 3.0
- **Motor driver** on I2C bus 7, address 0x34 (enable with `ENABLE_NAV2=true`)
- **Waveshare RoArm v2** on `/dev/ttyUSB0` at 115200 baud

## Configuration

Edit `docker/docker-compose.yml` environment variables:

```yaml
ENABLE_SLAM: "true"       # Visual SLAM odometry
ENABLE_YOLO: "true"       # YOLOv8 detection
ENABLE_BEHAVIOR: "true"   # State machine
ENABLE_NAV2: "false"      # Navigation stack (disabled by default)
```

## Monitor System

```bash
# Exec into running container
docker exec -it docker-vision-1 bash

# Check topics
ros2 topic list

# Monitor state machine
ros2 topic echo /robot/state

# Check sock detection
ros2 topic echo /sock/target_point_map
```

## Verify SLAM is Working

**Check SLAM status:**
```bash
docker exec docker-vision-1 bash -c "source /opt/vision_ws/install/setup.bash && ros2 topic echo /visual_slam/status --once"
```

**`vo_state` values:**
- `0` = NOT_READY (no camera input - check relay nodes)
- `1` = VISUAL_ONLY (receiving images, waiting for movement/features)
- `2` = TRACKING (fully operational ✓)

**Verify odometry publishing:**
```bash
docker exec docker-vision-1 bash -c "source /opt/vision_ws/install/setup.bash && timeout 3 ros2 topic hz /visual_slam/tracking/odometry"
```
- Should show ~15-30Hz when tracking
- If no output, move robot in circles with rotation for 20-30 seconds

**Check TF transforms:**
```bash
docker exec docker-vision-1 bash -c "source /opt/vision_ws/install/setup.bash && ros2 run tf2_ros tf2_echo map odom"
```
- If working: shows transform updates
- If "map does not exist": SLAM not tracking yet, keep moving robot

## Key Topics

- `/camera/*` - RealSense camera streams
- `/visual_slam/tracking/odometry` - SLAM odometry
- `/yolo/detections` - YOLO detection results
- `/sock/target_point_map` - 3D sock position in map frame
- `/robot/state` - Behavior state (WANDER, APPROACH_SOCK, etc.)
- `/cmd_vel` - Velocity commands (from behavior → Nav2)

## Architecture

```
RealSense → Visual SLAM → map→odom TF
         → YOLO → Sock Perception → 3D Point
                                  ↓
                          Behavior Manager
                                  ↓
                               Nav2 (optional)
```

All controlled by a single state machine in `behavior_manager_node.py`.

## Troubleshooting

**No camera topics:** Check USB connection (must be USB 3.0 blue port)
**No SLAM odometry:** Move camera around, ensure textured environment  
**No sock detections:** Verify model at `/models/clothes2.onnx` and `/models/clothes2.plan`
**TF errors:** Wait for SLAM to initialize and publish transforms
**"No valid depth in window":** clothes detected but no depth data available. This happens when:
  - clothes are too close to camera (< 20cm)
  - clothes are dark/fabric that absorbs infrared
  - clothes are lying flat on surface at steep angle
  - Try moving clothes 30-50cm away from camera with better IR reflection
