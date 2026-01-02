# SLAM Visualization Integration

## What Was Added

### Backend ([web_viewer_server_3d.py](src/web_viewer_server_3d.py))

**New Subscribers:**
- `/visual_slam/tracking/odometry` → Builds robot trajectory path
- `/visual_slam/vis/landmarks_cloud` → Map landmark points

**New WebSocket Endpoints:**
- `ws://host:8765/slam_trajectory` - Robot trajectory (JSON array of positions)
- `ws://host:8765/slam_pose` - Current robot pose (JSON with position + quaternion)
- `ws://host:8765/slam_landmarks` - Map landmarks (binary point cloud)

**Features:**
- Maintains last 1000 trajectory points
- Converts odometry to trajectory path
- Binary encoding for efficient landmark streaming
- Status reporting for all SLAM data streams

### Frontend ([viewer.html](src/viewer.html))

**New Tab: "SLAM Map"**

Visualizes 3 key SLAM components:

1. **Trajectory (Green Line)**
   - Shows complete path robot has traveled
   - Updates in real-time as robot moves
   - Can be cleared with "Clear Trajectory" button

2. **Current Pose (Blue Arrow)**
   - Arrow shows robot position and orientation
   - Updates at ~20 FPS
   - Arrow points in robot's forward direction

3. **Landmarks (White Points)**
   - Sparse map of environment features
   - These are the features SLAM uses for localization
   - Color-coded based on feature type

**Controls:**
- Separate connect buttons for each data stream
- Checkboxes to show/hide each component
- Reset view to default camera position
- Clear trajectory history

**Camera Controls:**
- Mouse drag: Rotate view
- Mouse wheel: Zoom
- Right-click drag: Pan

---

## Usage

### 1. Start the System

Make sure your container is running with SLAM enabled:

```bash
cd /home/taylor/workspaces/CleaningRobot/vision_production
docker compose -f docker/docker-compose.yml up
```

### 2. Launch the Web Viewer

Inside the container:

```bash
docker exec -it docker-vision-1 bash
ros2 run your_package web_viewer_server_3d.py
# Or if it's a script:
python3 /opt/vision/src/web_viewer_server_3d.py
```

### 3. Open the Viewer

Navigate to [viewer.html](src/viewer.html) in your browser (adjust IP address in the inputs).

### 4. Connect to SLAM

1. Click the **"SLAM Map"** tab
2. Click **"Connect Trajectory"** to see the robot's path
3. Click **"Connect Pose"** to see current position/orientation
4. Click **"Connect Landmarks"** to see the map

---

## What You'll See

### Typical SLAM Session

**Initial State:**
- Empty scene with grid and axes
- "0 points" in trajectory and landmarks

**As Robot Moves:**
- Green line grows showing path traveled
- Blue arrow moves and rotates with robot
- White points appear as SLAM maps new features

**During Loop Closure:**
- Trajectory may shift slightly as SLAM corrects drift
- More landmarks appear in revisited areas

---

## Troubleshooting

### No Trajectory Data
- Check: `ros2 topic hz /visual_slam/tracking/odometry`
- Ensure SLAM is tracking (move robot with good texture/features)
- Check WebSocket connection in browser console

### No Landmarks
- Check: `ros2 topic hz /visual_slam/vis/landmarks_cloud`
- Landmarks only publish when SLAM is actively mapping
- Ensure `enable_slam_visualization: True` in launch file

### Pose Arrow Not Moving
- Check: `ros2 topic echo /visual_slam/tracking/odometry`
- Verify SLAM is tracking (not lost)
- Check browser console for parse errors

### Poor Performance
- Landmarks are downsampled to 10k points max
- Trajectory limited to 1000 points
- Reduce update rate in JavaScript if needed

---

## Coordinate System

The SLAM visualization uses the ROS coordinate system:
- **X (Red axis)**: Forward
- **Y (Green axis)**: Left  
- **Z (Blue axis)**: Up

The camera defaults to a top-down view for easier navigation visualization.

---

## Next Steps

With SLAM visualization working, you can:

1. **Monitor localization quality** - Watch trajectory smoothness
2. **Verify map building** - See landmark density
3. **Debug loop closures** - Watch trajectory corrections
4. **Test navigation** - Visualize planned paths (when Nav2 is added)

Once you're satisfied with SLAM performance, we can add:
- **Nvblox** (3D occupancy mapping)
- **Nav2** (path planning & navigation)
- **Cost map visualization** (obstacles & free space)
