import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cam_w = LaunchConfiguration('cam_w')
    cam_h = LaunchConfiguration('cam_h')

    net_w = LaunchConfiguration('net_w')
    net_h = LaunchConfiguration('net_h')

    model_file_path = LaunchConfiguration('model_file_path')
    engine_file_path = LaunchConfiguration('engine_file_path')

    confidence_threshold = LaunchConfiguration('confidence_threshold')
    nms_threshold = LaunchConfiguration('nms_threshold')
    num_classes = LaunchConfiguration('num_classes')

    enable_slam = LaunchConfiguration('enable_slam')
    
    # Path to robot URDF
    urdf_path = '/opt/vision/urdf/robot.urdf.xacro'
    
    # Robot state publisher - publishes base_link -> camera_link
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', urdf_path]),
                value_type=str
            ),
            'use_sim_time': False,
        }],
    )

    container = ComposableNodeContainer(
        name='vision_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO'],
        composable_node_descriptions=[

            # ------------------------------------------------------------
            # RealSense camera outputs pre-rectified images
            # No need for separate rectify node - reduces latency & complexity
            # ------------------------------------------------------------

            # ------------------------------------------------------------
            # TensorRT — NO REMAPPINGS (NITROS handles connection in container)
            # ------------------------------------------------------------
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

            # ------------------------------------------------------------
            # YOLOv8 Decoder — NO REMAPPINGS (NITROS handles connection)
            # ------------------------------------------------------------
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
            ),

            # ------------------------------------------------------------
            # Visual SLAM — Consumes RGB, Depth, IMU, and TF
            # Publishes map -> odom transform
            # ------------------------------------------------------------
            ComposableNode(
                package='isaac_ros_visual_slam',
                plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
                name='visual_slam',
                parameters=[{
                    'enable_imu_fusion': True,
                    'enable_debug_mode': False,
                    'debug_dump_path': '/tmp/cuvslam',
                    'enable_slam_visualization': True,
                    'enable_landmarks_view': True,
                    'enable_observations_view': True,
                    'map_frame': 'map',
                    'odom_frame': 'odom',
                    'base_frame': 'base_link',
                    'enable_localization_n_mapping': True,
                    'publish_odom_to_base_tf': False,  # We use robot_localization for this
                    'publish_map_to_odom_tf': True,
                    'invert_odom_to_base_tf': False,
                    'denoise_input_images': False,
                    'rectified_images': True,
                    'image_buffer_size': 20,
                    # Relax synchronization tolerances for non-hardware-synced cameras
                    'img_jitter_threshold_ms': 100.0,  # Allow 100ms jitter (default 34ms)
                    'imu_jitter_threshold_ms': 50.0,   # Allow 50ms IMU jitter (default 10ms)
                }],
                remappings=[
                    ('visual_slam/image_0', '/camera/infra1/image_rect_raw'),
                    ('visual_slam/camera_info_0', '/camera/infra1/camera_info'),
                    ('visual_slam/image_1', '/camera/infra2/image_rect_raw'),
                    ('visual_slam/camera_info_1', '/camera/infra2/camera_info'),
                    ('visual_slam/imu', '/camera/imu'),
                ],
            ),
        ],
    )

    encoder_dir = get_package_share_directory('isaac_ros_dnn_image_encoder')
    encoder_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(encoder_dir, 'launch', 'dnn_image_encoder.launch.py')
        ),
        launch_arguments={
            # REQUIRED — must match raw color resolution
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
    )

    return LaunchDescription([
        DeclareLaunchArgument('cam_w', default_value='1280'),
        DeclareLaunchArgument('cam_h', default_value='720'),
        DeclareLaunchArgument('net_w', default_value='640'),
        DeclareLaunchArgument('net_h', default_value='640'),
        DeclareLaunchArgument('model_file_path', default_value='/models/socks2.onnx'),
        DeclareLaunchArgument('engine_file_path', default_value='/models/socks2.plan'),
        DeclareLaunchArgument('confidence_threshold', default_value='0.65'),
        DeclareLaunchArgument('nms_threshold', default_value='0.45'),
        DeclareLaunchArgument('num_classes', default_value='1'),
        DeclareLaunchArgument('enable_slam', default_value='true'),
        robot_state_publisher,
        container,
        encoder_launch,
    ])
