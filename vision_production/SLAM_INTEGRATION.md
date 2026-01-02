# Visual SLAM Integration Summary

## What Was Added

### 1. Isaac ROS Visual SLAM Package
- Added `ros-humble-isaac-ros-visual-slam=3.2.5-0jammy` to Dockerfile
- Added `ros-humble-robot-state-publisher` for TF publishing
- Added `ros-humble-xacro` for URDF processing

### 2. Robot URDF (`urdf/robot.urdf.xacro`)
- Defines `base_link` (robot center)
- Defines `camera_link` (RealSense D455 mount point)
- Fixed transform: base_link → camera_link (15cm forward by default)
- Includes all RealSense optical frames:
  - `camera_depth_optical_frame`
  - `camera_color_optical_frame`
  - `camera_infra1_optical_frame`
  - `camera_infra2_optical_frame`
  - `camera_imu_optical_frame`

**⚠️ IMPORTANT**: Adjust the camera mounting position in [urdf/robot.urdf.xacro](urdf/robot.urdf.xacro) line 28:
```xml
<origin xyz="0.15 0.0 0.0" rpy="0 0 0"/>
```
Change `xyz` values to match your actual camera position relative to robot center.

### 3. Updated Launch File
- Added `robot_state_publisher` node
- Added `isaac_ros_visual_slam` composable node to the container
- Visual SLAM configuration:
  - **Inputs**: Stereo infrared images + IMU
  - **Outputs**: `map → odom` TF transform
  - **IMU fusion**: Enabled for better tracking
  - **Visualization**: Enabled (landmarks, observations)

### 4. Updated RealSense Configuration
The entrypoint now launches RealSense with:
- ✅ Stereo infrared cameras (`enable_infra1`, `enable_infra2`)
- ✅ IMU (gyro @ 200Hz, accel @ 63Hz)
- ✅ Color + depth (existing)
- ✅ IMU fusion via `unite_imu_method:=1`

### 5. Docker Compose Updates
- Added `ENABLE_SLAM` environment variable
- Added persistent volume `slam_data` for cuVSLAM cache

---

## TF Tree Structure

```
map (published by visual_slam)
 └─ odom (published by visual_slam)
     └─ base_link (from robot_state_publisher or odometry)
         └─ camera_link (from robot_state_publisher)
             ├─ camera_depth_optical_frame
             ├─ camera_color_optical_frame  
             ├─ camera_infra1_optical_frame (used by SLAM)
             ├─ camera_infra2_optical_frame (used by SLAM)
             └─ camera_imu_optical_frame (used by SLAM)
```

**Note**: Visual SLAM publishes `map → odom` but NOT `odom → base_link`. You'll need:
- Option 1: Enable `publish_odom_to_base_tf: True` in visual_slam node
- Option 2: Use `robot_localization` to fuse wheel odometry + visual odometry
- Option 3: Your motor driver publishes `odom → base_link`

---

## Published Topics

### Visual SLAM Outputs
- `/visual_slam/tracking/odometry` - Visual odometry (nav_msgs/Odometry)
- `/visual_slam/tracking/vo_pose` - Visual pose (geometry_msgs/PoseStamped)
- `/visual_slam/tracking/vo_pose_covariance` - Pose with covariance
- `/visual_slam/vis/observations_cloud` - Feature observations (for debugging)
- `/visual_slam/vis/landmarks_cloud` - Map landmarks (for debugging)
- `/visual_slam/vis/loop_closure_cloud` - Loop closure points

### Existing Topics (YOLOv8)
- `/camera/color/image_raw`
- `/camera/depth/image_rect_raw`
- `/detections_output`

---

## Building and Running

### 1. Rebuild the Docker Image
```bash
cd /home/taylor/workspaces/CleaningRobot/vision_production
docker build -t vision_production:1.0 -f docker/Dockerfile .
```

### 2. Update Checksums (Production Practice)
```bash
sha256sum docker/Dockerfile launch/vision_bringup.launch.py \
  scripts/*.sh models/* urdf/* > checksums.txt
```

### 3. Start the Container
```bash
docker compose -f docker/docker-compose.yml up
```

### 4. Monitor SLAM Status
In another terminal:
```bash
docker exec -it docker-vision-1 bash

# Check if SLAM is publishing
ros2 topic hz /visual_slam/tracking/odometry

# View SLAM pose
ros2 topic echo /visual_slam/tracking/vo_pose

# Check TF tree
ros2 run tf2_tools view_frames
```

---

## Troubleshooting

### SLAM Not Tracking
- **Check infrared images**: `ros2 topic hz /camera/infra1/image_rect_raw`
- **Check IMU**: `ros2 topic hz /camera/imu`
- **Lighting**: SLAM needs good texture - avoid blank walls
- **Motion**: Start with slow, smooth movements

### TF Errors
- Verify URDF camera position matches physical mount
- Check `ros2 run tf2_ros tf2_echo map base_link`
- Ensure all frames are being published

### Performance Issues
- SLAM is GPU-intensive, monitor with `nvidia-smi`
- Reduce `image_buffer_size` if memory constrained
- Disable visualization if not needed: `enable_slam_visualization: False`

---

## Next Steps

1. ✅ **Visual SLAM** - DONE (this integration)
2. ⏭️ **Isaac ROS Nvblox** - 3D occupancy mapping
3. ⏭️ **Nav2** - Path planning and navigation
4. ⏭️ **Motor Driver Integration** - Connect cmd_vel to motors

Once you test SLAM, we can proceed with Nvblox!
