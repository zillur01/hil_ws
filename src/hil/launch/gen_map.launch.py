"""Plays a ros2 bag with lidar, camera and GNSS and builds a colored global pointcloud map on /projected
(view it in RViz, fixed frame `world`)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # TODO(USER): set the topic defaults to the topic names in your bags (ros2 bag info <bag>)
    return LaunchDescription([
        DeclareLaunchArgument('bag_path', description='ros2 bag directory with lidar, camera and GNSS'),
        DeclareLaunchArgument('rgb_topic', default_value='/zed/zed_node/left/image_rect_color'),
        DeclareLaunchArgument('camera_info_topic', default_value='/zed/zed_node/left/camera_info'),
        DeclareLaunchArgument('cloud_topic', default_value='/lidar1/velodyne_points'),
        ExecuteProcess(cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag_path'), '--clock', '--delay', '1'],
                       output='screen'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('hil'), 'launch', 'base.launch.py'])),
            launch_arguments={'use_sim_time': 'true'}.items()),
        Node(package='hil', executable='mapping_node.py', name='map_generation', output='screen',
             parameters=[{'use_sim_time': True}],
             # the node uses the placeholder names /rgb, /camera_info and /cloud
             remappings=[('/rgb', LaunchConfiguration('rgb_topic')),
                         ('/camera_info', LaunchConfiguration('camera_info_topic')),
                         ('/cloud', LaunchConfiguration('cloud_topic'))]),
    ])
