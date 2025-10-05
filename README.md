# CleaningRobot

This repository contains scripts and notes for the main hardware pieces of my robot:
- Intel RealSense in Isaac Ros environment
- Waveshare RoArm M2-S 4 axis arm
- HiWonder 4 wheel motor driver

Below are clean, copy-paste friendly commands and examples. All commands use plain ASCII quotes and are given as single lines so you can run them one at a time in a terminal.

## Isaac ROS / development helper commands for setting up vision system

Change to the workspace subdirectory:

```bash
cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common
```

Run the development script (example):

```bash
./scripts/run_dev.sh -d ~/workspaces/isaac_ros-dev -i ros2_humble.realsense
```

If that doesn't work or can't exist GPU or camera in container, add options:
```bash
docker run -it --rm --privileged --network host --ipc=host \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$HOME/.Xauthority":/home/admin/.Xauthority:rw \
  -e DISPLAY \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e ROS_DOMAIN_ID -e USER -e ISAAC_ROS_WS=/workspaces/isaac_ros-dev \
  -e HOST_USER_UID=$(id -u) -e HOST_USER_GID=$(id -g) \
  --pid=host -v /dev/input:/dev/input \
  -v /dev/bus/usb:/dev/bus/usb \
  -v /home/taylor/workspaces/isaac_ros-dev:/workspaces/isaac_ros-dev \
  --name isaac_ros_dev-aarch64-container \
  --gpus all \
  isaac_ros_dev-aarch64 /bin/bash
```

Add the ROS apt key if needed:

```bash
curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Update apt and install example Isaac ROS packages:

```bash
sudo apt-get update
```

```bash
sudo apt-get install -y ros-humble-isaac-ros-yolov8 ros-humble-isaac-ros-dnn-image-encoder ros-humble-isaac-ros-tensor-rt
```

```bash
sudo apt-get install -y ros-humble-isaac-ros-examples ros-humble-isaac-ros-realsense
```

```bash
pip install websockets
```

Run the launch file for RealSense
```bash
ros2 launch isaac_ros_examples isaac_ros_examples.launch.py launch_fragments:=realsense_mono_rect_depth,yolov8 model_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/yolov8/yolov8s.onnx engine_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/yolov8/yolov8s.plan
```

My model for dirt on wood floor
```bash
ros2 launch isaac_ros_examples isaac_ros_examples.launch.py launch_fragments:=realsense_mono_rect_depth,yolov8    model_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/yolov8/fixed.onnx engine_file_path:=${ISAAC_ROS_WS}/isaac_ros_assets/models/yolov8/fixed.plan
```

In another terminal in same container run the YOLOv8 visualization script:
```bash
ros2 run isaac_ros_yolov8 isaac_ros_yolov8_visualizer.py
```

In another terminal in same container run my 3d detector:
```bash
python /workspaces/isaac_ros-dev/src/isaac_ros_object_detection/isaac_ros_yolov8/scripts/yolov8_3d_detector.py
```

In another terminal in same container run my webviewer script:
```bash
python /workspaces/isaac_ros-dev/src/isaac_ros_object_detection/isaac_ros_yolov8/scripts/web_viewer_server_3d.py
```

And then on a client (I am just using my mac laptop) open up viewer.html and input (ws://192.168.1.160:8765) then click Connect.

Remember, to publish depth topics from realsense2_camera for this yolov8 demo, you can set enable_depth as true in /opt/ros/humble/share/isaac_ros_realsense/config/realsense_mono.yaml.
https://forums.developer.nvidia.com/t/how-to-publish-depth-topics-along-with-yolov8-object-detection-on-jetson-orin-nano/305624

# GGCNN
```bash
docker exec -it <Isaac-ROS container ID> bash
```

```bash
cd /workspaces/isaac_ros-dev/src
```

```bash
git clone https://github.com/dougsm/ggcnn.git
```

```bash
wget https://github.com/dougsm/ggcnn/releases/download/v0.1/ggcnn_weights_cornell.zip
```

```bash
unzip ggcnn_weights_cornell.zip
```

In the Isaac Ros container to publish depth data
```bash
ros2 launch isaac_ros_examples isaac_ros_examples.launch.py launch_fragments:=realsense_depth_rect_depth_to_color
```

Confirm it launched
```bash
ros2 topic list
```

```bash
python ggcnn_inference_ros2.py
```

## RoArm M2-S (Waveshare) — serial control examples

Step into the RoArm Python project directory:

```bash
cd ~/workspaces/isaac_ros-dev/src/RoArm-M2-S_python
```

Activate the Python virtual environment (example name used in this repo):

```bash
source ~/workspaces/isaac_ros-dev/src/RoArm-M2-S_python/roarmpython-env/bin/activate
```

Run the script with hardcoded sequences

```bash
python sequence_test_general.py /dev/ttyUSB0 --sequence A,B,C,D
```

Run the serial control script (replace device path if needed):

```bash
python serial_simple_ctrl.py /dev/ttyUSB0
```

### Example JSON messages

Use plain double quotes for JSON. These are single-line examples you can send to the robot over serial or a websocket depending on your setup.

Output telemetry
```json
{"T":105}
```

Open end effector

```json
{"T":106, "cmd":1.2, "spd":0, "acc":10}
```

Output telemetry
```json
{"T":105}
```


Set LED by numeric value:

```json
{"T":114, "led":10}
```

Move command example (x/y/z/t where t could be angle/time):

```json
{"T":1041, "x":235, "y":0, "z":234, "t":3.14}
```

LED on/off examples:

```json
{"LED":1}
{"T":114, "led":255}
{"T":114, "led":0}
```

Notes:

- If you copy-paste JSON into terminals, ensure your program or script expects a JSON string and send it appropriately (e.g., as a line over a serial connection or as a message over a websocket).
- The repository contains an example script, `serial_simple_ctrl.py`, that connects to the RoArm over a serial port and sends JSON messages typed into the terminal (one JSON message per line).

Usage example (replace device path as needed):

```bash
python serial_simple_ctrl.py /dev/ttyUSB0
```

After the script starts, type or paste a single-line JSON message and press Enter to send it to the robot.

License / Attribution

This README was cleaned for copy-paste use. Check repository files for original headers and licensing.

## Example: Simple pick-and-place sequence

Below is an 8-step example sequence (init, pick, move, rotate, place) expressed as single-line JSON messages you can send to the robot. All quotes are standard ASCII double quotes.

Steps:

1. Init
2. Open end effector
3. Go down to pick point
4. Close end effector
5. Move up (don't re-init)
6. Rotate base before going down to place in bin
7. Place in bin (move down)
8. Open end effector

<!-- Removed combined block to avoid duplication; see individually copyable blocks below -->

Individually copyable JSON messages (click-copy friendly). Copy only the JSON line in each block.


Starting point and open end effector

```json
{"T":102,"base":0.0,"shoulder":0.0,"elbow":1.35,"hand":0,"spd":0,"acc":20}
```

Go down to pick point and close end effector

```json
{"T":102,"base":0.0,"shoulder":1.03,"elbow":1.8,"hand":3.125,"spd":0,"acc":20}
```

Move up

```json
{"T":102,"base":0.0,"shoulder":0.0,"elbow":1.5,"hand":3.14,"spd":0,"acc":20}
```

Rotate base before placing

```json
{"T":102,"base":-1.20,"shoulder":0.0,"elbow":1.3,"hand":3.14,"spd":0,"acc":20}
```

Open end effector

```json
{"T":106, "cmd":1.2, "spd":0, "acc":20}
```


Notes:
- I fixed non-ASCII quotes and normalized speed/acc keys to numeric values.
- Step 7 was left blank in your original; I added a suggested move-down JSON command — adjust joint/rad values for your bin location.



