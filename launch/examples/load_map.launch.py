import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# FIXME: change this declaration of default map file name based on your actual setup
default_map_yaml_file = os.path.join(
    get_package_share_directory('rallycar'),
    'resources', 'maps', 'crane_500_map.yaml'
)


def generate_launch_description():
    """
    This function is by default called when executing ros2 launch ...
    This function must return a LaunchDescription object created from a list
    of launch_ros.actions
    """
    map_file = LaunchConfiguration('yaml_filename')
    map_file_arg = DeclareLaunchArgument(
        'yaml_filename',
        default_value = default_map_yaml_file,
        description = 'Full path of the yaml file to be loaded for a pgm map'
    )
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': map_file}
        ]
    )


    return LaunchDescription([
        map_file_arg,
        map_server_node,
    ])
