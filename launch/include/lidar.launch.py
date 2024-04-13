import os
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription

def get_share_file(package_name: str, *args: str) -> str:
    """Convert package-relative path to absolute path. Any additional args
    will be appended to the package_name, separated by '/'.

    Args:
        package_name (str): Package name.

    Returns:
        os.path: Absolute path.
    """
    return os.path.join(get_package_share_directory(package_name), *args)

def generate_launch_description():
    """
    This function is by default called when executing ros2 launch ...
    This function must return a LaunchDescription object created from a list
    of launch_ros.actions
    """
    hokuyo_node = Node(
        package='urg_node',
        name='hokuyo_lidar_driver_node',
        executable='urg_node_driver',
        output='screen',
        parameters=[
            get_share_file('rallycar', 'param', 'lidar.param.yaml'),
        ],
    )
    return LaunchDescription([
        hokuyo_node,
    ])
