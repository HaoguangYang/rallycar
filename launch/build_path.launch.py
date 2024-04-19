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


# FIXME: change these two declaration of default map/path file name based on your actual setup
default_map_file = get_share_file('rallycar', 'resources', 'maps', 'crane_500_map.yaml')
default_path_file = get_share_file('rallycar', 'resources', 'paths', 'crane_500_demo_path.yaml')


def generate_launch_description():
    """
    This function is by default called when executing ros2 launch ...
    This function must return a LaunchDescription object created from a list
    of launch_ros.actions
    """
    map_file = LaunchConfiguration('map_file')
    path_file = LaunchConfiguration('path_file')
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value = default_map_file,
        description = 'Full path of the yaml file to be loaded for a pgm map'
    )
    path_file_arg = DeclareLaunchArgument(
        'path_file',
        default_value = default_path_file,
        description = 'Full path of the yaml file to be saved as a desired trajectory, overwrites existing file'
    )
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        output='screen',
        parameters=[
            { 'yaml_filename': map_file, },
        ]
    )
    path_saver_node = Node(
        package='rallycar',
        executable='path_recorder.py',
        output='screen',
        parameters=[
            { 'file_name': path_file, },
        ]
    )
    rviz_interactive_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[
            {}
        ]
    )


    return LaunchDescription([
        map_file_arg,
        path_file_arg,
        map_server_node,
        path_saver_node,
        rviz_interactive_node,
    ])
