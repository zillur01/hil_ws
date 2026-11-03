"""Plays a ros2 bag and extracts synced images, depth maps and GNSS poses into <bag dir>_extracted/<bag name>/
(see save_data_node.py). ROS 1 .bag files must be converted first (README, "Bags")."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bag = LaunchConfiguration('input_bag_path')
    # TODO(USER): set the topic defaults to the ZED topic names in your bags (ros2 bag info <bag>)
    return LaunchDescription([
        DeclareLaunchArgument('input_bag_path', description='ros2 bag directory to extract'),
        DeclareLaunchArgument('rgb_topic', default_value='/zed/zed_node/left/image_rect_color'),
        DeclareLaunchArgument('depth_topic', default_value='/zed/zed_node/depth/depth_registered'),
        DeclareLaunchArgument('camera_info_topic', default_value='/zed/zed_node/left/camera_info'),
        ExecuteProcess(cmd=['ros2', 'bag', 'play', bag, '--clock', '--rate', '1', '--delay', '2'], output='screen'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('hil'), 'launch', 'base.launch.py'])),
            launch_arguments={'use_sim_time': 'true'}.items()),
        Node(package='hil', executable='save_data_node.py', name='save_data_node', output='screen',
             parameters=[{'input_bag_path': bag, 'use_sim_time': True}],
             # the node uses the placeholder names /rgb, /depth and /camera_info
             remappings=[('/rgb', LaunchConfiguration('rgb_topic')),
                         ('/depth', LaunchConfiguration('depth_topic')),
                         ('/camera_info', LaunchConfiguration('camera_info_topic'))]),
    ])
