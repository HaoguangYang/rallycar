#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


class ScanmatcherOdomRepubNode(Node):
    """
    Converts an updating Odometry message into a tf pointing to a specified
    baselink frame. A typical use case of this node is a helper to bridge a
    missing tf from odom to base_link. To achieve this functionality, the user
    should set the "updater_topic" parameter to the topic with a publisher of
    Odometry message. On the publisher side, the user should set the
    header.frame_id field of the Odometry message as "odom", and the pose
    field of the message being the integrated robot pose since it started moving.

    This node Subscribes to a specified topic with message type Odometry, and
    extracts the header, pose.position, and pose.orientation fields. This node
    then packs these fields into a TransformStamped message type and publishes
    to /tf.

    The user also needs to specify the initial values of the transform, if not
    identity.

    A guard of minium update frequency is set, such that when the update topic
    is not specified or not updating, old values are packed with a current
    timestamp and re-published at that guard frequency.
    """
    def __init__(self):
        super().__init__('scanmatcher_odom_repub_node')

        # Declare and acquire `odom_frame_name` parameter
        self.declare_parameter('scanmatcher_pose_topic', 'pose')
        self.declare_parameter('odom_topic', 'scanmatcher_odom')
        self.declare_parameter('odom_header_frame_id_override', '')
        self.declare_parameter('odom_child_frame_id', 'base_link')

        sub_topic = self.get_parameter('scanmatcher_pose_topic').value
        # Subscribe to the specified topic with PoseWithCovarianceStamped message
        # and call the callback function on each message
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            sub_topic,
            self.handle_pose_update,
            rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        )
        pub_topic = self.get_parameter('odom_topic').value
        self.odom_pub = self.create_publisher(Odometry, pub_topic,
                            rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value)
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = \
            self.get_parameter('odom_header_frame_id_override').value
        self.odom_msg.child_frame_id = self.get_parameter('odom_child_frame_id').value


    def handle_pose_update(self, msg):
        self.odom_msg.header.stamp = msg.header.stamp
        if not self.odom_msg.header.frame_id:
            self.odom_msg.header.frame_id = msg.header.frame_id
        self.odom_msg.pose = msg.pose

        # FIXME: we do not have twist computed here.
        self.odom_pub.publish(self.odom_msg)


def main():
    rclpy.init()
    node = ScanmatcherOdomRepubNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()


if __name__ == "__main__":
    main()
