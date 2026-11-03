from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # TODO(USER): set the defaults below to your ZED topic names (ros2 topic list | grep zed)
    args = [
        # config of the virtual objects: absolute path, or relative to
        # src/HIL_realtime_rendering/configs (e.g. gem/static_cone_30m.yaml)
        DeclareLaunchArgument('config_path', default_value='gem/static_cone_30m.yaml'),
        DeclareLaunchArgument('rgb_topic', default_value='/zed/zed_node/left/image_rect_color'),
        DeclareLaunchArgument('depth_topic', default_value='/zed/zed_node/depth/depth_registered'),
        # open OpenCV windows with the original and the inserted image
        DeclareLaunchArgument('show_debug', default_value='false'),
        # use the first received GNSS height as local height 0 (see README, first test)
        DeclareLaunchArgument('auto_origin_alt', default_value='false'),
        # optional placement of the pedestrian (actor_0) of a jaywalking config such as gem/jaywalker.yaml;
        # `ros2 run hil plan_pedestrian.py` prints a command with these filled in. nan = keep the config.
        DeclareLaunchArgument('actor_east', default_value='nan'),
        DeclareLaunchArgument('actor_north', default_value='nan'),
        DeclareLaunchArgument('actor_walk_heading_deg', default_value='nan'),
        DeclareLaunchArgument('trigger_distance', default_value='nan'),
        # true when playing a bag with `ros2 bag play --clock`
        DeclareLaunchArgument('use_sim_time', default_value='false'),
    ]
    node = Node(
        package='hil',
        executable='insertion_node.py',
        name='insertion',
        output='screen',
        parameters=[{
            'config_path': LaunchConfiguration('config_path'),
            'show_debug': LaunchConfiguration('show_debug'),
            'auto_origin_alt': LaunchConfiguration('auto_origin_alt'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            **{n: ParameterValue(LaunchConfiguration(n), value_type=float) for n in (
                'actor_east', 'actor_north', 'actor_walk_heading_deg', 'trigger_distance')},
        }],
        # the node uses the placeholder names /rgb and /depth
        remappings=[('/rgb', LaunchConfiguration('rgb_topic')), ('/depth', LaunchConfiguration('depth_topic'))],
    )
    return LaunchDescription(args + [node])
