# Local Setup Summary

## ✅ Using Local Web Viewers (NO cross-machine DDS)

This repository no longer uses FastDDS/CycloneDDS cross-machine discovery. Instead, visualization is done via the web viewers served by the container.

**Quick start:** Run the container and open `http://localhost:8080/slam_viewer.html` and `http://localhost:8080/nvblox_viewer.html` on the device or a browser that can reach the device's HTTP port.

---

## 🤖 On Jetson (Robot) - Running Locally

### What Changed
- Removed cross-machine DDS configurations and FastDDS profiles
- Reverted to local web-based visualization via container-hosted HTTP and rosbridge
- Docker no longer relies on host-networking for inter-machine discovery

### Steps to Apply

```bash
cd ~/workspaces/CleaningRobot

# Rebuild the container with new configuration (ALREADY DONE)
docker compose -f docker/docker-compose.yml build

# Start the robot system  
docker compose -f docker/docker-compose.yml up
```

**Wait ~30 seconds for initialization**, then move robot around to initialize SLAM.

### Verify Topics on Jetson

```bash
ros2 topic list | grep visual_slam
# Should show all SLAM topics
```

---

## Remote visualization is deprecated

Cross-machine PC visualization and DDS configuration have been removed from this repository. Use the container-hosted web viewers (`http://localhost:8080/slam_viewer.html`, `http://localhost:8080/nvblox_viewer.html`) for visualization. For debugging and topic inspection, exec into the container and use `ros2 topic list` or `ros2 topic echo`.

## 🧪 Testing Checklist

### ✓ PC can see topics
```bash
ros2 topic list | grep visual_slam
```
Expected output:
```
/visual_slam/initial_pose
/visual_slam/status
/visual_slam/tracking/odometry
/visual_slam/tracking/slam_path
...
```

### ✓ SLAM is tracking (after robot movement)
```bash
ros2 topic hz /visual_slam/tracking/odometry
```
Expected: 15-30 Hz

### ✓ Nvblox mesh is publishing
```bash
ros2 topic hz /nvblox_node/mesh
```
Expected: 1-5 Hz

### ✓ TF tree is available
```bash
ros2 run tf2_tools view_frames
# Opens PDF showing transform tree
```

---

## 🔧 Troubleshooting (Local)

- Check topics inside the running container:

```bash
docker exec -it docker-vision-1 bash
source /opt/vision_ws/install/setup.bash
ros2 topic list
```

- SLAM not initialized? Move the robot with rotation for ~30 seconds and check:

```bash
ros2 topic echo /visual_slam/status --once
# Look for vo_state: 2 (TRACKING)
```

- Viewer not loading? Ensure the container is running and the HTTP/WS ports are reachable (container exposes `8080` for the HTTP viewer and `9090` for rosbridge).

---

## 🎯 Quick Reference

| Machine | Role | Access |
|--------:|------:|--------|
| Robot (device) | Hosts container | `http://localhost:8080` (viewers), `ws://localhost:9090` (rosbridge) |
| Local browser | Visualization | Open `http://localhost:8080/slam_viewer.html` or `http://localhost:8080/nvblox_viewer.html` |

**Common Commands:**

```bash
# Topic check (inside container)
ros2 topic list

# View SLAM status
ros2 topic echo /visual_slam/status --once

# Restart container
docker compose -f docker/docker-compose.yml restart
```

---

## 📚 Additional Resources

- **Full PC Setup Guide:** [PC_VISUALIZATION_SETUP.md](PC_VISUALIZATION_SETUP.md)
- **Robot Documentation:** [README.md](README.md)
- **Nvblox Integration Details:** [NVBLOX_INTEGRATION.md](NVBLOX_INTEGRATION.md)

---

## 🚀 Next Steps

1. **On Jetson:** Rebuild and start container
2. **On PC:** Configure environment, test connection
3. **Launch RViz2:** Visualize SLAM and nvblox
4. **Move robot:** Initialize SLAM tracking

**You're all set!** 🎉
