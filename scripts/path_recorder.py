#!/usr/bin/env python3
import rclpy
import yaml
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalID

"""
The path server node. It works in either publish mode (default) or record mode.
In publish mode, it loads a path from `resources/paths` file, and publishes it
at `/desired_path` topic with `latch=True`. The `latch` option allows any future
subscribers to receive the message. The path will be published only once for a
static path.
In record mode, it subscribes to `/move_base_simple/goal` and records every
received message as sequential poses. A subscriber is set up at `/move_base/cancel`
to delete the last pose when requested. Upon exit, it saves the path to the
filename specified as a yaml file.
"""

def list_of_pose_dict_to_path_msg(inp):
    path = Path()
    path.header.frame_id = "map"
    path.poses = []
    for item in inp:
        p = PoseStamped()
        p.header.frame_id = "map"
        (p.pose.position.x, p.pose.position.y, p.pose.position.z) =
            (item['position']['x'], item['position']['y'], item['position']['z'])
        (p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w) =
            (item['orientation']['x'], item['orientation']['y'],
             item['orientation']['z'], item['orientation']['w'])
        path.poses.append(p)
    return path


class PathRecorder(Node):
    """The Path Recorder mode, invoked when running in record mode.
    """

    def __init__(self):
        """Class constructor that subscribes to the topic `/move_base_simple/goal` and saves the path.
        The `/move_base_simple/goal` topic is published by Rviz, and records user click on the UI.

        Args:
            file_name (str): path file name to save with.
        """
        super().__init__('path_recorder_node')
        self.declare_parameter('path_file')
        self.path_file = self.get_parameter('path_file').value
        self.path = []
        self.path_sub = self.create_subscription(
            PoseStamped, "/move_base_simple/goal", self.clicked_pose_callback,
            rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value)
        self.path_pub = self.create_publisher(
            Path, "/desired_path", rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value)
        self.path_rm_last_sub = self.create_subscription(
            GoalID, "/move_base/cancel", self.del_last_pose_callback,
            rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value
        )

    def clicked_pose_callback(self, msg):
        """Callback function that records the pose when the user clicks on the UI.

        Args:
            msg (PoseStamped): Message published by Rviz
        """
        self.path.append({
            'position': {'x': msg.pose.position.x, 'y': msg.pose.position.y, 'z': msg.pose.position.z},
            'orientation': {'x': msg.pose.orientation.x, 'y': msg.pose.orientation.y,
                            'z': msg.pose.orientation.z, 'w': msg.pose.orientation.w}
        })
        self.path_pub.publish(list_of_pose_dict_to_path_msg(self.path))

    def del_last_pose_callback(self, msg):
        """Callback function that deletes the last entered point from the record.

        Args:
            msg (actionlib_msgs/GoalID): Empty indicator
        """
        if len(self.path):
            self.path.pop()
        self.path_pub.publish(list_of_pose_dict_to_path_msg(self.path))

    def save_path(self):
        """Function to be executed during node shutdown. It saves the path to disk.
        """
        if len(self.path) == 0:
            return
        with open(self.path_file, 'w+') as f:
            yaml.dump(self.path, f)


def main():
    rclpy.init()
    node = PathRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.save_path()
    node.destroy_node()


if __name__ == '__main__':
    main()
