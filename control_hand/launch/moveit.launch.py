from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [

#   <include file="$(find-pkg-share allegro_moveit_config)/launch/demo.launch.py" />
            #include the moveit demo launch file 
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([FindPackageShare('allegro_moveit_config'),
                                               '/launch/demo.launch.py']),
            ),

            Node(
                package='control_hand',
                executable='moveit_controller',
                name='moveit_controller',
                output='screen',
            ), 
        ]
    )