from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import math

def generate_launch_description():
    namespace = "fbot"
    h = math.pi/2
    p = math.pi
    return LaunchDescription([
        Node(
            package='borg_spraymap',
            executable='joiner',
            name='joiner',
            output='screen',
        ),
        Node(
            package='borg_spraymap',
            executable='clouder',
            name='clouder',
            output='screen',
        ),
    ])
