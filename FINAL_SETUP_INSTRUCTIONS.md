# FINAL SETUP — Local Web Viewers

We removed cross-machine DDS/networking and reverted to local web-based visualization using the built-in viewers.

Quick steps ✅
1. Start the stack on the device (e.g., Jetson):

```bash
cd ~/workspaces/CleaningRobot
docker compose -f docker/docker-compose.yml up --build
```

2. Open the viewers in a browser on the same machine:

- SLAM viewer: `http://localhost:8080/slam_viewer.html`
- Nvblox viewer: `http://localhost:8080/nvblox_viewer.html`

3. Check rosbridge and topics (inside the container):

```bash
docker exec -it docker-vision-1 bash
source /opt/vision_ws/install/setup.bash
ros2 topic list
# rosbridge websocket: ws://localhost:9090
```

Notes & troubleshooting 💡
- Cross-machine DDS discovery and FastDDS/CycloneDDS unicast setups have been removed.
- If you must access the web viewer from another machine, use the device IP (e.g., `http://<device-ip>:8080/...`) and ensure firewall/port forwarding allows access.
- To save viewer settings, use the browser's Save/Load options or download the files locally.

You're set — the local HTML viewers provide immediate visualization without cross-machine networking.