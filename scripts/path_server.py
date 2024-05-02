#!/usr/bin/env python3
"""
File: path_recorder.py
Author: Haoguang Yang (yang1510@purdue.edu)
Date: 2024-04-29
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import yaml
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


def list_of_pose_dict_to_path_msg(inp: "list[dict]") -> Path:
    """Translates a list of poses (dictionary) into a nav_msgs/Path ROS2
    message

    Args:
        inp (list[dict]): input of a list of poses. Each pose is a dictionary
        with corresponding field names. Assumes poses are in "map" frame.

    Returns:
        Path: resultant nav_msgs/Path message, with all header.frame_id set to
        "map".
    """
    path = Path()
    path.header.frame_id = "map"
    path.poses = []
    for item in inp:
        p = PoseStamped()
        p.header.frame_id = "map"
        pos = p.pose.position
        pos.x, pos.y, pos.z = (
            item["position"]["x"],
            item["position"]["y"],
            item["position"]["z"],
        )
        ori = p.pose.orientation
        ori.x, ori.y, ori.z, ori.w = (
            item["orientation"]["x"],
            item["orientation"]["y"],
            item["orientation"]["z"],
            item["orientation"]["w"],
        )
        path.poses.append(p)
    return path


class PathServer(Node):
    """The Path Server node.
    It loads a path from the file specified by the `path_file` param, and
    publishes it at `/desired_path` topic with qos:
    QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST, depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL
    )
    The qos allows any future subscribers to receive the message exactly once
    for static paths.
    """

    def __init__(self):
        """Class constructor that loads the path from file and publishes it."""
        super().__init__("path_server_node")
        self.declare_parameter("path_file", "")
        file_name = self.get_parameter("path_file").value
        with open(file_name, "r") as f:
            inp = yaml.safe_load(f)
        # converts data structure loaded from yaml (list of dict) to ROS2 msg
        self.path = list_of_pose_dict_to_path_msg(inp)
        # override timestamp to the current system time
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.get_logger().info("Read from file: %s" % (file_name,))
        # here defines the special qos profile
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        # create the publisher and send the msg out
        self.path_pub = self.create_publisher(Path, "/desired_path", qos)
        self.path_pub.publish(self.path)


def main():
    rclpy.init()
    node = PathServer()
    try:
        # this line guards the process until Ctrl-C is pressed
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()


if __name__ == "__main__":
    main()
