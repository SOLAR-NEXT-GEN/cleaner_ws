from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    launch_description = LaunchDescription()
    package_name = 'solar_ros'

    lidar_read = Node(
        package = package_name,
        executable = 'lidar_distance_check.py',
        name = 'lidar'
    )
    
    # robot = Node(
    #     package = package_name,
    #     executable = 'robot_control.py',
    #     name = 'robot'
    # )

    robot_L = Node(
        package = package_name,
        executable = 'drive_solo_L.py',
        name = 'robot'
    )

    robot_R = Node(
        package = package_name,
        executable = 'drive_solo_R.py',
        name = 'robot'
    )

    mqtt = Node(
        package = package_name,
        executable = 'mqtt2ros.py',
        name = 'mqtt'
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("sllidar_ros2"),
                    "launch",
                    "custom_sllidar_c1_launch.py"
                )
            ]
        )
    )
    
    launch_description.add_action(lidar)
    launch_description.add_action(lidar_read)
    launch_description.add_action(robot_L)
    launch_description.add_action(robot_R)
    launch_description.add_action(mqtt)

    return launch_description