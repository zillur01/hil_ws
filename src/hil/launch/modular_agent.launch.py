from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # TODO(USER): set the defaults below to your ZED topic names (ros2 topic list | grep zed)
    args = [
        # waypoints recorded in the local frame (x, y, heading_deg), e.g. gem_gnss_control's my_track.csv
        # (recorded with the same origin as hil/origin.py), see README
        DeclareLaunchArgument('waypoint_csv_path', default_value='data/waypoints/my_track.csv'),
        DeclareLaunchArgument('rgb_topic', default_value='/zed/zed_node/left/image_rect_color'),
        DeclareLaunchArgument('depth_topic', default_value='/zed/zed_node/depth/depth_registered'),
        # leave empty to use src/HIL_modular_autonomy from the source tree (needs --symlink-install)
        DeclareLaunchArgument('agent_dir', default_value=''),
        # true: use the virtual-object-inserted image/depth from the insertion node instead of the camera
        DeclareLaunchArgument('use_inserted_data', default_value='false'),
        # true when playing a bag with `ros2 bag play --clock`
        DeclareLaunchArgument('use_sim_time', default_value='false'),
    ]

    def agent_node(rgb, depth, condition):
        return Node(
            package='hil',
            executable='modular_agent_node.py',
            name='modular_agent',
            output='screen',
            parameters=[{
                'waypoint_csv_path': LaunchConfiguration('waypoint_csv_path'),
                'agent_dir': LaunchConfiguration('agent_dir'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            # the node uses the placeholder names /rgb and /depth
            remappings=[('/rgb', rgb), ('/depth', depth)],
            condition=condition,
        )

    camera = agent_node(LaunchConfiguration('rgb_topic'), LaunchConfiguration('depth_topic'),
                        UnlessCondition(LaunchConfiguration('use_inserted_data')))
    inserted = agent_node('/insertion/rendered_image', '/insertion/rendered_depth',
                          IfCondition(LaunchConfiguration('use_inserted_data')))
    return LaunchDescription(args + [camera, inserted])
