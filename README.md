# Cleaning Robot - Vision Production

**Autonomous sock-collecting robot with behavior-driven architecture.**

## Quick Start

```bash
# Build
docker compose -f docker/docker-compose.yml build

# Run
docker compose -f docker/docker-compose.yml up
```

See [QUICKSTART.md](QUICKSTART.md) for details.

---

## Architecture

```
RealSense D455
  ├─ RGB → YOLOv8 → Sock Detections
  ├─ Aligned Depth → 3D Point Extraction
  ├─ Stereo IR → Visual SLAM → Odometry
  └─ IMU → SLAM Fusion

Sock Perception → 3D Target (map frame)
                     ↓
            Behavior Manager (State Machine)
                     ↓
                  Nav2 → /cmd_vel
                     ↓
            Motor Controller → I2C Motors

Arm Bridge ← Behavior Manager (pick/place services)
    ↓
Waveshare RoArm v2 (serial)
```

### Components

- **Visual SLAM**: Stereo odometry from infrared cameras + IMU fusion
- **YOLO Detection**: GPU-accelerated sock detection (Isaac ROS)
- **Sock Perception**: 3D target extraction with temporal filtering
- **Behavior Manager**: 6-state orchestration (WANDER→APPROACH→PICK→BASKET→PLACE→RECOVER)
- **Motor Controller**: Velocity PID control for I2C motor driver (bus 7, addr 0x34)
- **Arm Bridge**: Serial control of Waveshare RoArm v2 with hand-eye calibration
- **Nav2**: Navigation stack (optional, disabled by default)

### Key Design Principles

1. **One container, one launch** - No backgrounded processes
2. **Behavior orchestrates navigation** - State machine sends Nav2 goals
3. **Service-gated perception** - Enable/disable without killing nodes
4. **Navigation safety-first** - Robot never moves without behavior approval

---

## Project Structure

```
src/
├── robot_bringup/     # Unified launch, config, URDF
├── sock_perception/   # YOLO→3D pipeline
├── behavior_manager/  # State machine
├── motor_controller/  # Velocity PID for I2C motors
└── arm_bridge/        # Waveshare RoArm v2 control
```

All configuration is in ROS2 packages. See [QUICKSTART.md](QUICKSTART.md) for monitoring and troubleshooting.

## Configuration

Edit `docker/docker-compose.yml` environment variables:

```yaml
ENABLE_SLAM: "true"       # Visual SLAM odometry
ENABLE_YOLO: "true"       # YOLOv8 sock detection
ENABLE_BEHAVIOR: "true"   # Behavior state machine
ENABLE_NAV2: "false"      # Navigation stack
```

## Development

The workspace uses `--symlink-install`, so Python node changes don't require rebuild:

```bash
# Edit node
vim src/behavior_manager/behavior_manager/behavior_manager_node.py

# Restart container
docker compose -f docker/docker-compose.yml restart
```

Changes take effect immediately!

    optionally publishes: odom -> base_link

isaac_ros_nvblox
    consumes: depth + TF (map → camera)

Nav2
    consumes: odom + costmap + TF
    publishes: cmd_vel

Your driver
    consumes: cmd_vel
    talks to motors over I²C