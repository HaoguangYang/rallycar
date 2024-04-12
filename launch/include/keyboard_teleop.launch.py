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
    teleop_node = Node(
        package='rallycar',
        name='keyboard_teleop_node',
        executable='rally_teleop_keyboard.py',
        output='screen',
        parameters=[
            get_share_file('rallycar', 'param', 'keyboard_teleop.param.yaml'),
        ],
    )
    return LaunchDescription([
        teleop_node,
    ])
