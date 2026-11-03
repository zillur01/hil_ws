"""Projects lidar points onto the camera image (/projected_vis) to check the lidar-camera calibration
(T_lidar_rgb in hil_common/globals.py). Live, or from a bag with bag_path:=<ros2 bag>."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bag = LaunchConfiguration('bag_path')
    use_bag = PythonExpression(["'", bag, "' != ''"])
    # TODO(USER): set the topic defaults to your ZED / lidar topic names
    return LaunchDescription([
        DeclareLaunchArgument('bag_path', default_value=''),
        DeclareLaunchArgument('rgb_topic', default_value='/zed/zed_node/left/image_rect_color'),
        DeclareLaunchArgument('camera_info_topic', default_value='/zed/zed_node/left/camera_info'),
        DeclareLaunchArgument('cloud_topic', default_value='/lidar1/velodyne_points'),
        ExecuteProcess(cmd=['ros2', 'bag', 'play', bag, '--clock', '--delay', '1'], output='screen',
                       condition=IfCondition(use_bag)),
        Node(package='hil', executable='lidar2cam_node.py', name='lidar2cam', output='screen',
             parameters=[{'use_sim_time': ParameterValue(use_bag, value_type=bool)}],
             # the node uses the placeholder names /rgb, /camera_info and /cloud
             remappings=[('/rgb', LaunchConfiguration('rgb_topic')),
                         ('/camera_info', LaunchConfiguration('camera_info_topic')),
                         ('/cloud', LaunchConfiguration('cloud_topic'))]),
    ])
