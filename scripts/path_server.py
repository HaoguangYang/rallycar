#!/usr/bin/env python3
import rclpy
import yaml
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

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


class PathServer(Node):
    """The Path Server mode, invoked when running in publish mode.
    """

    def __init__(self):
        """Class constructor that loads the path from file and publishes it.
        """
        super().__init__('path_server_node')
        self.declare_parameter('path_file')
        file_name = self.get_parameter('path_file').value
        with open(file_name, 'r') as f:
            inp = yaml.safe_load(f)
        self.path = list_of_pose_dict_to_path_msg(inp)
        self.path.header.stamp = self.get_clock().now().to_msg()
        print(self.path)
        qos = rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value
        qos.history = rclpy.qos.HistoryPolicy.KEEP_LAST.value
        qos.durability = rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL.value
        self.path_pub = self.create_publisher(Path, "/desired_path", qos)
        self.path_pub.publish(self.path)


def main():
    rclpy.init()
    node = PathServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()


if __name__ == '__main__':
    main()
