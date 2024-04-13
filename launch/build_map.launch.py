import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

def generate_launch_description():
    """
    This function is by default called when executing ros2 launch ...
    This function must return a LaunchDescription object created from a list
    of launch_ros.actions
    """
    # include other launch files
    hardware_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_share_file('rallycar', 'launch', 'rallycar_hardware.launch.py')
        )
    )

    teleop_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_share_file('rallycar', 'launch', 'include', 'keyboard_teleop.launch.py')
        )
    )

    # we can create nodes and put into the launch description as well
    slam_odomless_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_node',
        output='screen',
        namespace='',
        parameters=[
          get_share_file('rallycar', 'param', 'slam_toolbox_odomless.param.yaml'),
          {
            'use_lifecycle_manager': false,
            'use_sim_time': false
          }
        ],
    )

    return LaunchDescription([
        hardware_driver_launch,
        teleop_node_launch,
        slam_odomless_node,
    ])
