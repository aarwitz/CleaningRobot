FROM isaac_ros_dev-aarch64:r36.4.4_ir3.2_frozen

RUN apt-get update && apt-get install -y \
    ros-humble-isaac-ros-realsense=3.2.5-0jammy \
    ros-humble-isaac-ros-image-proc=3.2.10-0jammy \
    ros-humble-isaac-ros-dnn-image-encoder=3.2.10-0jammy \
    ros-humble-isaac-ros-tensor-rt=3.2.10-0jammy \
    ros-humble-isaac-ros-yolov8=3.2.5-0jammy \
    ros-humble-isaac-ros-segment-anything=3.2.10-0jammy \
    ros-humble-isaac-ros-triton=3.2.10-0jammy \
    usbutils \
 && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir \
    websockets==12.0 \
    timm \
    git+https://github.com/ChaoningZhang/MobileSAM.git

RUN ls /usr/local/lib/librealsense* >/dev/null

COPY launch/vision_bringup.launch.py /opt/vision/vision_bringup.launch.py
COPY scripts /opt/vision/scripts
COPY src /opt/vision/src
COPY models /models
COPY checksums.txt /opt/vision/checksums.txt

RUN chmod +x /opt/vision/scripts/*.sh

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=0

ENTRYPOINT ["/opt/vision/scripts/entrypoint.sh"]
