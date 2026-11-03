"""Records waypoints (local x, y, heading_deg CSV) while driving, or from a bag with bag_path:=<ros2 bag>.
Prefer gem_gnss_control's record_waypoints on the car (README); this one writes the same format."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bag = LaunchConfiguration('bag_path')
    use_bag = PythonExpression(["'", bag, "' != ''"])
    return LaunchDescription([
        DeclareLaunchArgument('csv_name', default_value='recorded_track'),
        # empty = src/hil/data/waypoints of the source checkout (needs colcon build --symlink-install)
        DeclareLaunchArgument('output_dir', default_value=''),
        # optional ros2 bag to record the waypoints from instead of the live car
        DeclareLaunchArgument('bag_path', default_value=''),
        ExecuteProcess(cmd=['ros2', 'bag', 'play', bag, '--clock', '--delay', '1'], output='screen',
                       condition=IfCondition(use_bag)),
        Node(package='hil', executable='save_waypoint_node.py', name='save_waypoint_node', output='screen',
             parameters=[{
                 'filename': LaunchConfiguration('csv_name'),
                 'output_dir': LaunchConfiguration('output_dir'),
                 'use_sim_time': ParameterValue(use_bag, value_type=bool),
             }]),
    ])
