"""Publishes the car pose as tf world -> gps (and an RViz axis marker), from /navsatfix + /insnavgeod."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # true when playing a bag with `ros2 bag play --clock`
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='hil', executable='gps_tf_pub_node.py', name='gps_tf_pub_node', output='log',
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ])
