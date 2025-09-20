# CleaningRobot

This repository contains scripts and notes for working with the Waveshare RoArm M2-S and some Isaac ROS hints.

Below are clean, copy-paste friendly commands and examples. All commands use plain ASCII quotes and are given as single lines so you can run them one at a time in a terminal.

## Isaac ROS / development helper commands

Change to the workspace subdirectory:

```bash
cd workspaces/isaac_ros_ws/src/isaac_ros_common
```

Run the development script (example):

```bash
./scripts/run_dev.sh -d ~/workspaces/isaac_ros_ws -i ros2_humble.realsense
```

Add the ROS apt key (single command):

```bash
curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Update apt and install example Isaac ROS packages:

```bash
sudo apt-get update
sudo apt-get install -y ros-humble-isaac-ros-yolov8 ros-humble-isaac-ros-dnn-image-encoder ros-humble-isaac-ros-tensor-rt
```

## RoArm M2-S (Waveshare) — serial control examples

Step into the RoArm Python project directory:

```bash
cd ~/workspaces/isaac_ros-dev/src/RoArm-M2-S_python
```

Activate the Python virtual environment (example name used in this repo):

```bash
source roarmpython-env/bin/activate
```

Run the serial control script (replace device path if needed):

```bash
python serial_simple_ctrl.py /dev/ttyUSB0
```

### Example JSON messages

Use plain double quotes for JSON. These are single-line examples you can send to the robot over serial or a websocket depending on your setup.

Set a timestamp-like value:

```json
{"T":100}
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
- The repository contains example scripts such as `serial_simple_ctrl.py` and `cleanest.py` in various folders. Inspect those scripts for expected message formats.

License / Attribution

This README was cleaned for copy-paste use. Check repository files for original headers and licensing.

