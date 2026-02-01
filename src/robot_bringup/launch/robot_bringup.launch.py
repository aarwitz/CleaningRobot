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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution, Command
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode, ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression

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
        'enable_nav2', default_value='false',
        description='Enable Nav2 navigation stack'
    )

    enable_arm_arg = DeclareLaunchArgument(
        'enable_arm', default_value='true',
        description='Enable Waveshare RoArm bridge node'
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
        'enable_color', default_value='false',
        description='Enable RGB/color stream on RealSense'
    )

    enable_depth_arg = DeclareLaunchArgument(
        'enable_depth', default_value='false',
        description='Enable depth stream on RealSense'
    )
    
    align_depth_arg = DeclareLaunchArgument(
        'align_depth_enable', default_value='true',
        description='Enable depth-to-color alignment in RealSense'
    )
    
    enable_imu_arg = DeclareLaunchArgument(
        'enable_imu', default_value='false',
        description='Enable IMU fusion for Visual SLAM'
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
        'model_file_path', default_value='/models/clothes2.onnx',
        description='Path to YOLO ONNX model'
    )
    
    engine_file_arg = DeclareLaunchArgument(
        'engine_file_path', default_value='/models/clothes2.plan',
        description='Path to TensorRT engine file'
    )
    
    conf_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold', default_value='0.65',
        description='YOLO confidence threshold'
    )
    
    nms_threshold_arg = DeclareLaunchArgument(
        'nms_threshold', default_value='0.45',
        description='YOLO NMS threshold'
    )
    
    num_classes_arg = DeclareLaunchArgument(
        'num_classes', default_value='1',
        description='Number of YOLO classes'
    )
    
    # Get argument values
    camera_only = LaunchConfiguration('camera_only')
    enable_slam = LaunchConfiguration('enable_slam')
    enable_yolo = LaunchConfiguration('enable_yolo')
    enable_behavior = LaunchConfiguration('enable_behavior')
    enable_nav2 = LaunchConfiguration('enable_nav2')
    enable_arm = LaunchConfiguration('enable_arm')
    cam_w = LaunchConfiguration('cam_w')
    cam_h = LaunchConfiguration('cam_h')
    # depth_profile removed; use literal profile string below
    enable_color = LaunchConfiguration('enable_color')
    enable_depth = LaunchConfiguration('enable_depth')
    net_w = LaunchConfiguration('net_w')
    net_h = LaunchConfiguration('net_h')
    model_file_path = LaunchConfiguration('model_file_path')
    engine_file_path = LaunchConfiguration('engine_file_path')
    confidence_threshold = LaunchConfiguration('confidence_threshold')
    nms_threshold = LaunchConfiguration('nms_threshold')
    num_classes = LaunchConfiguration('num_classes')
    align_depth_enable = LaunchConfiguration('align_depth_enable')
    enable_imu = LaunchConfiguration('enable_imu')
    
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
            'infra1.profile': '640,480,30',
            'infra2.profile': '640,480,30',
            'depth_module.enable_auto_exposure': 'true',
            'depth_module.emitter_enabled': '0',
            'enable_infra_emitter': 'false',
            'emitter_enabled': '0',

            'enable_sync': 'false',

            'rgb_camera.profile': [
                cam_w, TextSubstitution(text=','), cam_h, TextSubstitution(text=',30')
            ],

            'enable_gyro': enable_imu,
            'enable_accel': enable_imu,
            'gyro_fps': '200',
            'accel_fps': '200',
            'unite_imu_method': '2',
            }
        .items()
    )

    
    # 2. Visual SLAM - with relay nodes for topic remapping
    # Isaac ROS Visual SLAM expects specific topic names, so we relay camera topics
    slam_relay_left_image = Node(
        package='topic_tools',
        executable='relay',
        name='slam_relay_left_image',
        arguments=['/camera/infra1/image_raw', '/visual_slam/image_0'],
        condition=IfCondition(PythonExpression([
            "'", enable_slam, "' == 'true' and '",
            camera_only, "' == 'false'"]))
    )
    
    slam_relay_right_image = Node(
        package='topic_tools',
        executable='relay',
        name='slam_relay_right_image',
        arguments=['/camera/infra2/image_raw', '/visual_slam/image_1'],
        condition=IfCondition(PythonExpression([
            "'", enable_slam, "' == 'true' and '",
            camera_only, "' == 'false'"]))
    )
    
    slam_relay_left_info = Node(
        package='topic_tools',
        executable='relay',
        name='slam_relay_left_info',
        arguments=['/camera/infra1/camera_info', '/visual_slam/camera_info_0'],
        condition=IfCondition(PythonExpression([
                "'", enable_slam, "' == 'true' and '",
                camera_only, "' == 'false'"]))
    )
    
    slam_relay_right_info = Node(
        package='topic_tools',
        executable='relay',
        name='slam_relay_right_info',
        arguments=['/camera/infra2/camera_info', '/visual_slam/camera_info_1'],
        condition=IfCondition(PythonExpression([
                "'", enable_slam, "' == 'true' and '",
                camera_only, "' == 'false'"]))
    )

    # (No republish relays — driver will publish image_raw when configured)
    
    visual_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('isaac_ros_visual_slam'),
                'launch',
                'isaac_ros_visual_slam.launch.py'
            ])
        ]),
        launch_arguments={
            'enable_imu_fusion': enable_imu,
            'enable_rectified_pose': 'True',
            'rectified_images': 'False',
            'enable_slam_visualization': 'True',
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'input_base_frame': 'camera_link',
            'publish_odom_to_base_tf': 'True',
            'publish_map_to_odom_tf': 'True',
        }.items(),
        condition=IfCondition(PythonExpression([
                "'", enable_slam, "' == 'true' and '",
                camera_only, "' == 'false'"]))
    )
    
    # 3. YOLOv8 detection - composable node container
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
            'track_width': 0.256,
            'cmd_per_mps': 240.0,
            'control_rate': 20.0,
        }],
        output='screen',
        condition=IfCondition(enable_nav2)  # Only when Nav2 is enabled
    )
    
    # 7. Arm bridge (Waveshare RoArm v2)
    arm_bridge_node = Node(
        package='arm_bridge',
        executable='arm_bridge_node',
        name='arm_bridge_node',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'baud_rate': 115200,
            'enable_arm': True,
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
        condition=IfCondition(enable_nav2)
    )
    
    # Assemble launch description
    return LaunchDescription([
        # Arguments
        camera_only_arg,
        enable_slam_arg,
        enable_yolo_arg,
        enable_behavior_arg,
        enable_nav2_arg,
        cam_w_arg,
        cam_h_arg,
        net_w_arg,
        net_h_arg,
        model_file_arg,
        engine_file_arg,
        conf_threshold_arg,
        nms_threshold_arg,
        num_classes_arg,
        
        # Nodes/launches
        realsense_launch,
        # Ensure camera auto-exposure is applied at startup (fallback)
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'param', 'set', '/camera/camera', 'depth_module.enable_auto_exposure', 'true'],
                    output='screen'
                )
            ]
        ),
        slam_relay_left_image,
        slam_relay_right_image,
        slam_relay_left_info,
        slam_relay_right_info,
        visual_slam_launch,
        vision_container,
        yolo_encoder_launch,
        clothes_perception_node,
        behavior_manager_node,
        motor_controller_node,
        arm_bridge_node,
        nav2_launch,
        robot_state_publisher_node,
    ])
