#!/usr/bin/env python3
"""
Unified Robot Bringup Launch File

Single launch file that brings up the entire robot system:
- RealSense camera
- Visual SLAM
- YOLOv8 perception
- Clothes perception node
- Behavior manager
- Arm bridge (stub)
- Robot state publisher

NO background bash jobs - everything is a proper ROS2 node/launch include.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, PythonExpression
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode, ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare arguments
    camera_only_arg = DeclareLaunchArgument(
        'camera_only',
        default_value='false',
        description='Launch only the RealSense camera (no SLAM, no perception)'
)

    enable_slam_arg = DeclareLaunchArgument(
        'enable_slam', default_value='true',
        description='Enable Visual SLAM'
    )
    
    enable_yolo_arg = DeclareLaunchArgument(
        'enable_yolo', default_value='true',
        description='Enable YOLOv8 detection'
    )
    
    enable_behavior_arg = DeclareLaunchArgument(
        'enable_behavior', default_value='true',
        description='Enable behavior manager'
    )
    
    enable_nav2_arg = DeclareLaunchArgument(
        'enable_nav2', default_value='true',
        description='Enable Nav2 navigation stack'
    )

    enable_arm_arg = DeclareLaunchArgument(
        'enable_arm', default_value='true',
        description='Enable Waveshare RoArm bridge node'
    )
    
    enable_visualization_arg = DeclareLaunchArgument(
        'enable_visualization', default_value='true',
        description='Enable rosbridge for web-based visualization'
    )
    
    enable_nvblox_arg = DeclareLaunchArgument(
        'enable_nvblox', default_value='true',
        description='Enable nvblox 3D reconstruction'
    )
    
    cam_w_arg = DeclareLaunchArgument(
        'cam_w', default_value='640',
        description='Camera width'
    )
    
    cam_h_arg = DeclareLaunchArgument(
        'cam_h', default_value='480',
        description='Camera height'
    )

    # depth_profile removed; using fixed profile string to avoid missing launch config

    enable_color_arg = DeclareLaunchArgument(
        'enable_color', default_value='true',
        description='Enable RGB/color stream on RealSense'
    )

    enable_depth_arg = DeclareLaunchArgument(
        'enable_depth', default_value='true',
        description='Enable depth stream on RealSense'
    )
    
    align_depth_arg = DeclareLaunchArgument(
        'align_depth_enable', default_value='true',
        description='Enable depth-to-color alignment in RealSense'
    )
    
    enable_imu_arg = DeclareLaunchArgument(
        'enable_imu', default_value='True',
        description='Enable IMU fusion for Visual SLAM'
    )

    camera_fps_arg = DeclareLaunchArgument(
        'camera_fps', default_value='90',
        description='Camera frame rate (fps) for all streams (infrared, depth, color)'
    )

    net_w_arg = DeclareLaunchArgument(
        'net_w', default_value='640',
        description='YOLO network input width'
    )
    
    net_h_arg = DeclareLaunchArgument(
        'net_h', default_value='640',
        description='YOLO network input height'
    )
    
    model_file_arg = DeclareLaunchArgument(
        'model_file_path', default_value='/models/yolov8s.onnx',
        description='Path to YOLO ONNX model'
    )
    
    engine_file_arg = DeclareLaunchArgument(
        'engine_file_path', default_value='/models/yolov8s.plan',
        description='Path to TensorRT engine file'
    )

    force_engine_update_arg = DeclareLaunchArgument(
        'force_engine_update', default_value='true',
        description='Force TensorRT engine rebuild from ONNX at startup'
    )
    
    conf_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold', default_value='0.15',
        description='YOLO confidence threshold'
    )
    
    nms_threshold_arg = DeclareLaunchArgument(
        'nms_threshold', default_value='0.5',
        description='YOLO NMS threshold'
    )
    
    num_classes_arg = DeclareLaunchArgument(
        'num_classes', default_value='80',
        description='Number of YOLO classes (80 for COCO/yolov8s, 1 for custom single-class)'
    )
    
    # Get argument values
    camera_only = LaunchConfiguration('camera_only')
    enable_slam = LaunchConfiguration('enable_slam')
    enable_yolo = LaunchConfiguration('enable_yolo')
    enable_behavior = LaunchConfiguration('enable_behavior')
    enable_nav2 = LaunchConfiguration('enable_nav2')
    enable_arm = LaunchConfiguration('enable_arm')
    enable_visualization = LaunchConfiguration('enable_visualization')
    enable_nvblox = LaunchConfiguration('enable_nvblox')
    cam_w = LaunchConfiguration('cam_w')
    cam_h = LaunchConfiguration('cam_h')
    # depth_profile removed; use literal profile string below
    enable_color = LaunchConfiguration('enable_color')
    enable_depth = LaunchConfiguration('enable_depth')
    net_w = LaunchConfiguration('net_w')
    net_h = LaunchConfiguration('net_h')
    model_file_path = LaunchConfiguration('model_file_path')
    engine_file_path = LaunchConfiguration('engine_file_path')
    force_engine_update = LaunchConfiguration('force_engine_update')
    confidence_threshold = LaunchConfiguration('confidence_threshold')
    nms_threshold = LaunchConfiguration('nms_threshold')
    num_classes = LaunchConfiguration('num_classes')
    align_depth_enable = LaunchConfiguration('align_depth_enable')
    enable_imu = LaunchConfiguration('enable_imu')
    camera_fps = LaunchConfiguration('camera_fps')
    
    # 1. RealSense camera
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ]),
        launch_arguments={
            'enable_color': enable_color,
            'enable_depth': enable_depth,

            'enable_infra1': 'true',
            'enable_infra2': 'true',

            # CRITICAL: disable per-stream rectification
            'enable_infra1_rectification': 'false',
            'enable_infra2_rectification': 'false',

            # Infra profiles (width,height,fps) and enable auto exposure persistently
            'infra1.profile': ['640,480,', camera_fps],
            'infra2.profile': ['640,480,', camera_fps],
            
            # CRITICAL: Force depth resolution to match infra for nvblox
            'depth_module.profile': ['640,480,', camera_fps],
            'depth_module.enable_auto_exposure': 'true',
            'depth_module.emitter_enabled': '0',
            'enable_infra_emitter': 'false',
            'emitter_enabled': '0',
            
            # Enable depth-to-color alignment for nvblox
            'align_depth.enable': align_depth_enable,

            'enable_sync': 'true',
            'depth_module.global_time_enabled': 'true',

            # Set explicit color profile to match infra/depth resolution
            'rgb_camera.profile': ['640,480,', camera_fps],

            'enable_gyro': enable_imu,
            'enable_accel': enable_imu,
            'gyro_fps': '200',
            'accel_fps': '200',
            'unite_imu_method': '2',
            }
        .items()
    )

    
    # 2. Visual SLAM - direct topic remapping (no relay nodes for zero latency)
    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            'enable_image_denoising': False,
            'rectified_images': False,
            'enable_imu_fusion': enable_imu,
            'gyro_noise_density': 0.000244,
            'gyro_random_walk': 0.000019393,
            'accel_noise_density': 0.001862,
            'accel_random_walk': 0.003,
            'calibration_frequency': 200.0,
            'image_jitter_threshold_ms': 120.00,
            'enable_rectified_pose': True,
            'enable_slam_visualization': True,
            'enable_landmarks_view': True,
            'enable_observations_view': True,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'imu_frame': 'camera_gyro_optical_frame',
            'publish_odom_to_base_tf': True,
            'publish_map_to_odom_tf': True,
            'camera_optical_frames': [
                'camera_infra1_optical_frame',
                'camera_infra2_optical_frame',
            ],
        }],
        remappings=[
            ('visual_slam/image_0', '/camera/infra1/image_rect_raw'),
            ('visual_slam/camera_info_0', '/camera/infra1/camera_info'),
            ('visual_slam/image_1', '/camera/infra2/image_rect_raw'),
            ('visual_slam/camera_info_1', '/camera/infra2/camera_info'),
            ('visual_slam/imu', '/camera/imu'),
        ],
        condition=IfCondition(enable_slam)
    )
    
    visual_slam_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[visual_slam_node],
        output='screen',
        condition=IfCondition(enable_slam)
    )
    
    # 2b. Nvblox 3D reconstruction - builds volumetric map from depth + SLAM odometry
    nvblox_node = ComposableNode(
        name='nvblox_node',
        package='nvblox_ros',
        plugin='nvblox::NvbloxNode',
        parameters=[{
            'global_frame': 'map',
            'voxel_size': 0.05,  # 5cm voxels
            'esdf': True,  # Enable ESDF for navigation
            'esdf_2d': True,  # Enable 2D slice for Nav2 costmap
            # The costmap reads /nvblox_node/static_map_slice, so the SLICE BAND
            # that matters is static_mapper.esdf_slice_{min,max}_height. NOTE: the
            # 'esdf_2d_min_height'/'esdf_2d_max_height' keys this build does NOT
            # declare were silently ignored, so nvblox used its default min=0.0 =
            # the floor. The slice band is in the map frame where z=0 is the floor
            # (base_link sits at floor level); the level camera at 0.2 m sees the
            # ground entering its FOV ~0.36 m ahead, so min=0.0 flattened that
            # floor into the costmap as a lethal "wall" ~0.3 m in front, blocking
            # every forward goal. min=0.10 m clears the floor (plus voxel noise)
            # so only real obstacles taller than 10 cm count. Clothes/socks on the
            # floor are pick targets, not nav obstacles, so excluding them is
            # correct. max=1.0 captures furniture/walls.
            'static_mapper.esdf_slice_min_height': 0.10,
            'static_mapper.esdf_slice_max_height': 1.0,
            'distance_slice': True,
            'mesh': True,  # Enable mesh output for visualization
            'max_tsdf_update_hz': 10.0,
            'max_color_update_hz': 5.0,
            'max_mesh_update_hz': 5.0,
            'max_esdf_update_hz': 2.0,
            'tsdf_integrator_max_integration_distance_m': 10.0,
            'mesh_integrator_min_weight': 1e-4,
            'mesh_integrator_weld_vertices': True,
        }],
        remappings=[
            # NOTE: this nvblox build names its inputs camera_0/* (multi-camera
            # support). The old remap keys ('depth/image', ...) did NOT match, so
            # nvblox silently subscribed to nonexistent /camera_0/* topics and
            # received zero depth -> empty map. The keys must be camera_0/*.
            #
            # Use RAW depth (depth-module clock), not depth-aligned-to-color.
            # nvblox places depth via TF (use_tf_transforms=True), looking up
            # map->depth_frame at the depth timestamp. cuVSLAM only publishes TF
            # on the infra/depth-module clock; color-aligned depth is stamped
            # ~67ms ahead of it, so lookups would need extrapolation into the
            # future. Raw depth shares SLAM's exact clock (measured 0.0ms).
            ('camera_0/depth/image', '/camera/depth/image_rect_raw'),
            ('camera_0/depth/camera_info', '/camera/depth/camera_info'),
            ('camera_0/color/image', '/camera/color/image_raw'),
            ('camera_0/color/camera_info', '/camera/color/camera_info'),
            ('pose', '/visual_slam/tracking/vo_pose'),
            ('pointcloud', '/nvblox/pointcloud'),
        ],
        condition=IfCondition(enable_nvblox)
    )
    
    nvblox_container = ComposableNodeContainer(
        name='nvblox_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[nvblox_node],
        output='screen',
        condition=IfCondition(enable_nvblox)
    )
    
    # 3. YOLOv8 detection - Isaac ROS composable node container
    vision_container = ComposableNodeContainer(
        name='vision_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO'],
        composable_node_descriptions=[
            # TensorRT inference node
            ComposableNode(
                package='isaac_ros_tensor_rt',
                plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
                name='tensor_rt',
                parameters=[{
                    'model_file_path': model_file_path,
                    'engine_file_path': engine_file_path,
                    'input_binding_names': ['images'],
                    'input_tensor_names': ['input_tensor'],
                    'output_binding_names': ['output0'],
                    'output_tensor_names': ['output_tensor'],
                    'force_engine_update': False,
                }],
            ),
            # YOLOv8 decoder node
            ComposableNode(
                package='isaac_ros_yolov8',
                plugin='nvidia::isaac_ros::yolov8::YoloV8DecoderNode',
                name='yolov8_decoder',
                parameters=[{
                    'tensor_name': 'output_tensor',
                    'confidence_threshold': confidence_threshold,
                    'nms_threshold': nms_threshold,
                    'num_classes': num_classes,
                }],
                remappings=[
                    ('detections_output', '/yolo/detections'),
                ],
            ),
        ],
        condition=IfCondition(enable_yolo)
    )
    
    # DNN Image Encoder launch (preprocessor for YOLO)
    encoder_dir = get_package_share_directory('isaac_ros_dnn_image_encoder')
    yolo_encoder_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(encoder_dir, 'launch', 'dnn_image_encoder.launch.py')
        ),
        launch_arguments={
            'input_image_width': cam_w,
            'input_image_height': cam_h,
            'input_encoding': 'rgb8',
            'image_input_topic': '/camera/color/image_raw',
            'camera_info_input_topic': '/camera/color/camera_info',
            'tensor_output_topic': '/tensor_pub',
            'attach_to_shared_component_container': 'True',
            'component_container_name': '/vision_container',
            'network_image_width': net_w,
            'network_image_height': net_h,
            'image_mean': '[0.0, 0.0, 0.0]',
            'image_stddev': '[1.0, 1.0, 1.0]',
        }.items(),
        condition=IfCondition(enable_yolo)
    )
    
    # 4. Clothes perception node
    clothes_perception_node = Node(
        package='clothes_perception',
        executable='clothes_perception_node',
        name='clothes_perception_node',
        parameters=[{
            'confidence_threshold': 0.7,
            'temporal_filter_size': 5,
            'depth_window_size': 7,
            'max_depth_m': 5.0,
            'min_depth_m': 0.2,
            'camera_frame': 'camera_color_optical_frame',
            'map_frame': 'map',
            'rate_hz': 5.0,
        }],
        output='screen'
        ,
        condition=IfCondition(enable_yolo)
    )

    # 5. Behavior manager (always launch)
    behavior_manager_node = Node(
        package='behavior_manager',
        executable='behavior_manager_node',
        name='behavior_manager_node',
        parameters=[{
            'wander_radius_m': 3.0,
            'wander_timeout_s': 30.0,
            'clothes_confidence_threshold': 0.7,
            'clothes_stable_frames_required': 5,
            'clothes_stable_time_s': 2.0,
            'grasp_offset_m': 0.30,
            'approach_stop_distance_m': 0.35,
            'goal_update_threshold_m': 0.10,
            'goal_update_max_rate_s': 2.0,
            'basket_x': 0.0,
            'basket_y': 0.0,
            'basket_z': 0.3,
            'approach_timeout_s': 60.0,
            'pick_timeout_s': 30.0,
            'wander_perception_rate_hz': 3.0,
            'approach_perception_rate_hz': 8.0,
        }],
        output='screen',
        condition=IfCondition(PythonExpression([
        "'", camera_only, "' == 'false' and '",
        enable_behavior, "' == 'true'"
        ]))
    )
    
    # 6. Motor controller (velocity control for Nav2 cmd_vel)
    motor_controller_node = Node(
        package='motor_controller',
        executable='motor_controller_node',
        name='motor_controller_node',
        parameters=[{
            'bus_id': 7,
            'i2c_addr': 0x34,
            'ticks_per_meter': 18940.0,
            'wheel_geom_L': 0.25,  # lx+ly (half-wheelbase+half-track); scales omega
            'cmd_per_mps': 240.0,
            'control_rate': 20.0,
        }],
        output='screen',
        condition=IfCondition(enable_nav2)  # Only when Nav2 is enabled
    )
    
    # 7. Arm bridge (Waveshare RoArm v2) – autonomous pick-and-place
    arm_bridge_node = Node(
        package='arm_bridge',
        executable='arm_bridge_node',
        name='arm_bridge_node',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'baud_rate': 115200,
            'enable_arm': True,
            'dry_run': False,
            'stable_frames': 4,
            'stable_max_drift_px': 40.0,
            'cooldown_s': 5.0,
            'min_depth_m': 0.15,
            'max_depth_m': 0.60,
        }],
        output='screen',
        condition=IfCondition(enable_arm)
    )
    
    # 7. Nav2 (optional, for full autonomy)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': PathJoinSubstitution([
                FindPackageShare('robot_bringup'),
                'config',
                'nav2_params.yaml'
            ]),
        }.items(),
        condition=IfCondition(enable_nav2)
    )
    
    # 8. Robot state publisher (static transforms from URDF)
    urdf_file = PathJoinSubstitution([
        FindPackageShare('robot_bringup'),
        'urdf',
        'robot.urdf.xacro'
    ])
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', urdf_file]),
                value_type=str
            ),
            'use_sim_time': False,
        }],
        output='screen',
        condition=IfCondition(enable_slam)
    )
    
    # 9. Rosbridge server for web-based visualization
    rosbridge_server = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{
            'port': 9090,
            'address': '0.0.0.0',
        }],
        output='screen',
        condition=IfCondition(enable_visualization)
    )
    
    # 10. HTTP server for web viewer
    http_server = ExecuteProcess(
        cmd=['python3', '-m', 'http.server', '8080', '--directory', '/opt/vision_ws'],
        output='screen',
        condition=IfCondition(enable_visualization)
    )
    
    # Assemble launch description
    return LaunchDescription([
        # Arguments
        camera_only_arg,
        enable_slam_arg,
        enable_yolo_arg,
        enable_behavior_arg,
        enable_nav2_arg,
        enable_arm_arg,
        enable_visualization_arg,
        enable_nvblox_arg,
        cam_w_arg,
        cam_h_arg,
        enable_color_arg,
        enable_depth_arg,
        align_depth_arg,
        enable_imu_arg,
        camera_fps_arg,
        net_w_arg,
        net_h_arg,
        model_file_arg,
        engine_file_arg,
        force_engine_update_arg,
        conf_threshold_arg,
        nms_threshold_arg,
        num_classes_arg,
        
        # Nodes/launches
        realsense_launch,
        # Ensure camera auto-exposure is applied at startup (fallback)
        # The auto-exposure is what was reducing my acquistion rate
        # Has to be applied at startup after camera is initialized
        # https://nvidia-isaac-ros.github.io/v/release-3.1/troubleshooting/hardware_setup.html
        # links to the issue here: https://github.com/realsenseai/realsense-ros/issues/2507#issuecomment-1411214372
        # 
        # The single param-set at +3s used to DIE (exit 1): the RealSense node's
        # parameter service isn't up that early (USB enumeration runs well past
        # 3s), so "ros2 param set" hit "node not found". When this re-apply
        # fails, auto-exposure never re-enables, infra drops to ~15Hz (long
        # exposure), and cuVSLAM starves and drifts. Retry until it lands.
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c',
                         'for i in $(seq 1 40); do '
                         '  if ros2 param set /camera/camera '
                         'depth_module.enable_auto_exposure true; then '
                         '    echo "[auto-exposure] re-applied OK on attempt $i"; exit 0; '
                         '  fi; '
                         '  sleep 2; '
                         'done; '
                         'echo "[auto-exposure] FAILED to re-apply after retries"; exit 1'],
                    output='screen'
                )
            ]
        ),
        visual_slam_container,
        nvblox_container,
        vision_container,
        yolo_encoder_launch,
        clothes_perception_node,
        behavior_manager_node,
        motor_controller_node,
        arm_bridge_node,
        nav2_launch,
        robot_state_publisher_node,
        rosbridge_server,
        http_server,
    ])
