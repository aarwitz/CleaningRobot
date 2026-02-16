# Nvblox Integration Summary

## What Was Added

### 1. **Nvblox 3D Volumetric Reconstruction**
   - Package: `ros-humble-isaac-ros-nvblox` (already in Dockerfile)
   - Builds TSDF/ESDF volumetric maps from RGB-D camera + SLAM odometry
   - Outputs: 3D mesh, 2D distance map for Nav2 costmap integration
   - Voxel size: 5cm (configurable)

### 2. **Configuration Files Modified**

#### docker-compose.yml
- Added `ENABLE_NVBLOX: "false"` environment variable
- Note: Requires depth+color enabled to function

#### Dockerfile
- Updated to copy both `slam_viewer.html` and `nvblox_viewer.html`

#### entrypoint.sh
- Added ENABLE_NVBLOX display in configuration output
- Passes enable_nvblox flag to launch file

#### robot_bringup.launch.py
- Added `enable_nvblox_arg` launch argument
- Created nvblox_node ComposableNode with configuration:
  * Subscribes to: depth image, color image, camera info, SLAM odometry
  * Publishes: `/nvblox_node/mesh`, `/nvblox_node/map_slice`, `/nvblox_node/static_esdf_pointcloud`
  * Parameters: voxel_size=0.05m, ESDF enabled, 2D slice for Nav2
- Added nvblox_container to launch description

### 3. **Viewers Created**

#### slam_viewer.html (renamed from viewer.html)
- Shows Isaac ROS Visual SLAM output
- Topics: path, landmarks, odometry, status
- Green theme (#00ff88)

#### nvblox_viewer.html (new)
- Shows 3D volumetric reconstruction mesh
- Topics: `/nvblox_node/mesh`, `/visual_slam/tracking/odometry`
- Blue theme (#00ccff)
- Features:
  * Multiple rendering modes (Phong, basic, wireframe, normals)
  * Statistics: triangle count, mesh update rate, map volume
  * Robot pose visualization
  * Auto-follow camera mode

### 4. **README Documentation**
- Updated configuration section with ENABLE_NVBLOX flag
- Added note: nvblox requires ENABLE_DEPTH=true, ENABLE_COLOR=true, ALIGN_DEPTH=true
- Added both viewer URLs to "Verify SLAM is Working" section
- Updated "What's Running" to include nvblox

## How to Use

### Enable Nvblox
Edit `docker/docker-compose.yml`:
```yaml
ENABLE_SLAM: "true"       # Required for odometry
ENABLE_NVBLOX: "true"     # Enable 3D reconstruction
ENABLE_DEPTH: "true"      # Required: depth stream
ENABLE_COLOR: "true"      # Required: RGB stream for textured mesh
ALIGN_DEPTH: "true"       # Required: align depth to color frame
```

### Access Viewers
- **SLAM Viewer**: `http://<device-ip>:8080/slam_viewer.html`
- **Nvblox Viewer**: `http://<device-ip>:8080/nvblox_viewer.html`

### Expected Behavior
1. Container starts with SLAM + nvblox nodes
2. Visual SLAM initializes and starts tracking (move camera in circles)
3. Nvblox begins building 3D mesh from depth+color data
4. Initial mesh appears after 10-30 seconds
5. Mesh incrementally grows as robot explores environment

### ROS Topics
```bash
# Check nvblox mesh publishing
ros2 topic hz /nvblox_node/mesh

# Check nvblox 2D costmap slice
ros2 topic echo /nvblox_node/map_slice --once

# Check ESDF for navigation
ros2 topic echo /nvblox_node/static_esdf_pointcloud --once
```

## Architecture

```
RealSense D455
  ├─ Stereo IR (infra1/infra2) → Visual SLAM → Odometry + TF (map→odom→base_link)
  ├─ Depth Stream (aligned) ────┐
  └─ Color Stream (RGB)  ────────┼─→ Nvblox → 3D Mesh + ESDF → Nav2 Costmap
                                 │           (subscribes to SLAM odometry for pose)
                                 └─→ slam_viewer.html
                                     nvblox_viewer.html
```

## Performance Notes

- **SLAM**: Uses stereo IR cameras, no depth needed
- **Nvblox**: Uses aligned depth + color, requires SLAM for pose
- **GPU Load**: Both SLAM and nvblox run on GPU, expect ~70-80% utilization
- **Mesh Update Rate**: ~5 Hz typical, depends on scene complexity
- **Memory**: Nvblox uses ~500MB-2GB RAM depending on explored volume

## Troubleshooting

**Nvblox not publishing mesh:**
- Check `ENABLE_DEPTH` and `ENABLE_COLOR` are both `true`
- Verify SLAM is tracking (`vo_state: 2`)
- Move camera to generate odometry data
- Check depth stream: `ros2 topic hz /camera/aligned_depth_to_color/image_raw`

**Grey viewer, no mesh visible:**
- Open browser console (F12) to check for JS errors
- Verify rosbridge connection status
- Wait 30 seconds for initial mesh blocks to accumulate
- Try changing render mode to "Wireframe" to see structure

**High latency/lag:**
- Reduce mesh update rate in launch file: `max_mesh_update_hz: 2.0`
- Increase voxel size for coarser map: `voxel_size: 0.10` (10cm)

## Future Enhancements

- [ ] Integrate nvblox ESDF with Nav2 costmap
- [ ] Add colored mesh from RGB stream
- [ ] Save/load mesh maps for persistent mapping
- [ ] Add dynamic object detection/filtering
- [ ] Implement loop closure with mesh matching
