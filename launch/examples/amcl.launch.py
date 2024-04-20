import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl_node',
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory('rallycar'), 'param', 'amcl.param.yaml'),
        ]
    )
    return LaunchDescription([
        amcl_node,
    ])
