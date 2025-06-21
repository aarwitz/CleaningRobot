# CleaningRobot


- cd workspaces/isaac_ros_ws/src/isaac_ros_common
- ./scripts/run_dev.sh -d ~/workspaces/isaac_ros_ws -i ros2_humble.realsense
- curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
- sudo apt-get update
- sudo apt-get install -y ros-humble-isaac-ros-yolov8 ros-humble-isaac-ros-dnn-image-encoder ros-humble-isaac-ros-tensor-rt
