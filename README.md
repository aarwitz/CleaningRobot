# CleaningRobot


- cd workspaces/isaac_ros_ws/src/isaac_ros_common
- ./scripts/run_dev.sh -d ~/workspaces/isaac_ros_ws -i ros2_humble.realsense
- curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
- sudo apt-get update
- sudo apt-get install -y ros-humble-isaac-ros-yolov8 ros-humble-isaac-ros-dnn-image-encoder ros-humble-isaac-ros-tensor-rt


For controlling the waveshare roarm m2-s:
(roarmpython-env) taylor@ubuntu:~/workspaces/isaac_ros-dev/src/RoArm-M2-S_python$ python serial_simple_ctrl.py /dev/ttyUSB0
{"T":1041,"x":235,"y":0,"z":234,"t":3.14}
Received: {"T":1041,"x":235,"y":0,"z":234,"t":3.14}
Received: 
{“T”:100}
{"LED":1}
Received: {"LED":1}
Received: 
{"LED":1}
^[[D
{"LED":1}

Received: {"LED":1}
Received: 
Received: {"LED":1}
Received: 
{"T":114,"led":255}
Received: {"T":114,"led":255}
Received: 
{"T":114,"led":0}      
Received: {"T":114,"led":0}
Received: 
