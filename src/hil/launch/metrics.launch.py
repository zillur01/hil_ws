"""Experiment metrics (time to goal, obstacle distance at first IL brake, latencies)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # waypoints recorded in the local frame (x, y, heading_deg), see README; the last one is the goal
        DeclareLaunchArgument('waypoint_csv_path', default_value='data/waypoints/my_track.csv'),
        DeclareLaunchArgument('experiment_name', default_value='test'),
        # true when playing a bag with `ros2 bag play --clock`
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='hil', executable='metrics_node.py', name='metrics', output='screen',
             parameters=[{
                 'waypoint_csv_path': LaunchConfiguration('waypoint_csv_path'),
                 'experiment_name': LaunchConfiguration('experiment_name'),
                 'use_sim_time': LaunchConfiguration('use_sim_time'),
             }]),
    ])
