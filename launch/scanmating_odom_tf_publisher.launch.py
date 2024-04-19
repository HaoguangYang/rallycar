from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    This function is by default called when executing ros2 launch ...
    This function must return a LaunchDescription object created from a list
    of launch_ros.actions
    """

    scanmatching_odom_node = Node(
        package='rallycar',
        executable='scanmatcher_odom_repub.py',
        name='scanmatcher_odom_repub_node',
        output='screen',
        parameters=[
            {
                'scanmatcher_pose_topic': '/pose',
                'odom_topic': '/odom',
                'odom_header_frame_id_override': 'odom',
                'odom_child_frame_id': 'base_link',
            }
        ],
    )

    odom_tf_node = Node(
        package='rallycar',
        executable='odom_tf_publisher.py',
        name='odom_tf_publisher_node',
        output='screen',
        parameters=[
            {
                'init_source_frame_name': 'odom',
                'target_frame_name': 'base_link',
                'init_tf_pose': [0., 0., 0., 0., 0., 0.],
                'updater_topic': '/odom',
                # broadcast tf at >= 40Hz
                'min_tf_broadcast_frequency': 40.0,
            },
        ],
    )

    return LaunchDescription([
        scanmatching_odom_node,
        odom_tf_node,
    ])
