FROM isaac_ros_dev-aarch64:r36.4.4_ir3.2_frozen

RUN apt-get update && apt-get install -y \
    ros-humble-isaac-ros-realsense=3.2.5-0jammy \
    ros-humble-isaac-ros-image-proc=3.2.10-0jammy \
    ros-humble-isaac-ros-dnn-image-encoder=3.2.10-0jammy \
    ros-humble-isaac-ros-tensor-rt=3.2.10-0jammy \
    ros-humble-isaac-ros-yolov8=3.2.5-0jammy \
    ros-humble-isaac-ros-visual-slam \
    ros-humble-isaac-ros-nvblox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    usbutils \
    i2c-tools \
 && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir websockets==12.0 smbus-cffi pyserial

RUN ls /usr/local/lib/librealsense* >/dev/null

# Create workspace and copy source
WORKDIR /opt/vision_ws
COPY src /opt/vision_ws/src

# Build workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Copy models and entrypoint
COPY models /models
COPY scripts/entrypoint.sh /opt/vision_ws/entrypoint.sh
RUN chmod +x /opt/vision_ws/entrypoint.sh

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=0

ENTRYPOINT ["/opt/vision_ws/entrypoint.sh"]
