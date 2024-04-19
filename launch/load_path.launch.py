import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_share_file(package_name: str, *args: str) -> str:
    """Convert package-relative path to absolute path. Any additional args
    will be appended to the package_name, separated by '/'.

    Args:
        package_name (str): Package name.

    Returns:
        os.path: Absolute path.
    """
    return os.path.join(get_package_share_directory(package_name), *args)


# FIXME: change this declaration of default map file name based on your actual setup
default_path_file = get_share_file('rallycar', 'resources', 'paths', 'crane_500_default_path.yaml')


def generate_launch_description():
    """
    This function is by default called when executing ros2 launch ...
    This function must return a LaunchDescription object created from a list
    of launch_ros.actions
    """
    path_file = LaunchConfiguration('path_file')
    path_file_arg = DeclareLaunchArgument(
        'path_file',
        default_value = default_path_file,
        description = 'Full path of the yaml file to be loaded as a desired trajectory'
    )
    path_server_node = Node(
        package='rallycar',
        executable='path_server.py',
        output='screen',
        parameters=[
            { 'file_name': path_file, },
        ]
    )

    return LaunchDescription([
        path_file_arg,
        path_server_node,
    ])
