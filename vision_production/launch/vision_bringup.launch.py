import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    cam_w = LaunchConfiguration('cam_w')
    cam_h = LaunchConfiguration('cam_h')

    net_w = LaunchConfiguration('net_w')
    net_h = LaunchConfiguration('net_h')

    model_file_path = LaunchConfiguration('model_file_path')
    engine_file_path = LaunchConfiguration('engine_file_path')

    confidence_threshold = LaunchConfiguration('confidence_threshold')
    nms_threshold = LaunchConfiguration('nms_threshold')

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
                }],
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
        DeclareLaunchArgument('model_file_path', default_value='/models/yolov8s.onnx'),
        DeclareLaunchArgument('engine_file_path', default_value='/models/yolov8s.plan'),
        DeclareLaunchArgument('confidence_threshold', default_value='0.25'),
        DeclareLaunchArgument('nms_threshold', default_value='0.45'),
        container,
        encoder_launch,
    ])
