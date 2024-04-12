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
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_share_file('rallycar', 'launch', 'include', 'lidar.launch.py')
        )
    )

    static_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_share_file('rallycar', 'launch', 'include', 'static_tf.launch.py')
        )
    )

    # find the serial port in the normal way
    serial_port = '/dev/ttyACM0'
    if not os.path_exists(serial_port):
        serial_port = '/dev/ttyUSB0'

    # we can create nodes and put into the launch description as well
    rallycar_driver_node = Node(
        package='rallycar',
        name='rallycar_driver',
        executable='rallycar_driver_node',
        output='screen',
        parameters=[
            # elements within the parameter list can be either a string pointing to a yaml file
            get_share_file('rallycar', 'param', 'rallycar_driver.param.yaml'),
            # ... or a dictionary of key-value pairs
            {"serial_port_fd": serial_port},
            # in case of conflicting definitions, the one defined later will override.
        ],
    )

    return LaunchDescription([
        lidar_launch,
        static_tf_launch,
        rallycar_driver_node,
    ])
