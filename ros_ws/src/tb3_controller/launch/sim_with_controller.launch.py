"""Start the TurtleBot3 world in Gazebo Harmonic and the obstacle_avoider controller.

    ros2 launch tb3_controller sim_with_controller.launch.py
    ros2 launch tb3_controller sim_with_controller.launch.py world:=house controller_delay:=15.0
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    tb3 = get_package_share_directory('turtlebot3_gazebo')
    world = LaunchConfiguration('world')
    world_launch = PythonExpression([
        "'", os.path.join(tb3, 'launch'), "/turtlebot3_' + '", world, "' + '.launch.py'"])

    controller = Node(
        package='tb3_controller',
        executable='obstacle_avoider',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='world',
                              description='world | house | dqn_stage1 ... (turtlebot3_<world>.launch.py)'),
        DeclareLaunchArgument('controller_delay', default_value='10.0',
                              description='seconds to wait for Gazebo before starting the controller'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(world_launch)),
        TimerAction(period=LaunchConfiguration('controller_delay'), actions=[controller]),
    ])
