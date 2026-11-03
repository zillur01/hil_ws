"""Imitation-learning agent (needs the HIL_imitation_learning submodule, see il_agent_node.py).
Output: /il/pred_brake (pacmod2_msgs/SystemCmdFloat), /il/desired_speed, /il/latency."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # TODO(USER): set the default below to your ZED topic name (ros2 topic list | grep zed)
    args = [
        DeclareLaunchArgument('rgb_topic', default_value='/zed/zed_node/left/image_rect_color'),
        # true: use the virtual-object-inserted image from the insertion node instead of the camera
        DeclareLaunchArgument('use_inserted_data', default_value='false'),
        # leave empty to use src/HIL_imitation_learning from the source tree (needs --symlink-install)
        DeclareLaunchArgument('il_dir', default_value=''),
        # true when playing a bag with `ros2 bag play --clock`
        DeclareLaunchArgument('use_sim_time', default_value='false'),
    ]

    def agent_node(rgb, condition):
        return Node(
            package='hil', executable='il_agent_node.py', name='il_agent', output='screen',
            parameters=[{'il_dir': LaunchConfiguration('il_dir'),
                         'use_sim_time': LaunchConfiguration('use_sim_time')}],
            # the node uses the placeholder name /rgb
            remappings=[('/rgb', rgb)],
            condition=condition)

    inserted = LaunchConfiguration('use_inserted_data')
    return LaunchDescription(args + [
        agent_node(LaunchConfiguration('rgb_topic'), UnlessCondition(inserted)),
        agent_node('/insertion/rendered_image', IfCondition(inserted)),
    ])
