"""Plays a ros2 bag at half speed and writes <bag dir>_extracted/<bag name>/synced.csv for imitation
learning (see save_data_node_IL.py). Everything stops when the node exits, 5 s after the bag ends."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bag = LaunchConfiguration('input_bag_path')
    # TODO(USER): set the topic defaults to the ZED topic names in your bags (ros2 bag info <bag>)
    return LaunchDescription([
        DeclareLaunchArgument('input_bag_path', description='ros2 bag directory to extract'),
        DeclareLaunchArgument('rgb_topic', default_value='/zed/zed_node/left/image_rect_color'),
        DeclareLaunchArgument('depth_topic', default_value='/zed/zed_node/depth/depth_registered'),
        ExecuteProcess(cmd=['ros2', 'bag', 'play', bag, '--clock', '--rate', '0.5', '--delay', '1'], output='screen'),
        Node(package='hil', executable='save_data_node_IL.py', name='save_data_node', output='screen',
             parameters=[{'input_bag_path': bag, 'use_sim_time': True}],
             # the node uses the placeholder names /rgb and /depth
             remappings=[('/rgb', LaunchConfiguration('rgb_topic')), ('/depth', LaunchConfiguration('depth_topic'))],
             on_exit=Shutdown()),
    ])
