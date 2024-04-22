#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


class ScanmatcherOdomRepubNode(Node):
    """
    Receives a message of type 'geometry_msgs/PoseWithCovarianceStamped', and
    copy its values over to publish a message of type 'nav_msgs/Odometry'. Only
    the pose fields contain valid values.

    Coordinate frame and timestamp handling: copies the timestamp from the
    received message. If parameter "odom_header_frame_id_override" is not set,
    it also copies the header.frame_id from the received message. The
    child_frame_id of the Odometry is set with parameter "odom_child_frame_id".

    This piece of code can be used as a bare-minimal example of sensor fusion or
    interpolation.
    """
    def __init__(self):
        super().__init__('scanmatcher_odom_repub_node')

        # Declare and acquire `odom_frame_name` parameter
        self.declare_parameter('scanmatcher_pose_topic', 'pose')
        self.declare_parameter('odom_topic', 'scanmatcher_odom')
        self.declare_parameter('odom_header_frame_id', '')
        self.declare_parameter('odom_child_frame_id', 'base_link')

        pub_topic = self.get_parameter('odom_topic').value
        self.odom_pub = self.create_publisher(Odometry, pub_topic,
                            rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value)
        # initialize the message to be published
        self.odom_msg = Odometry()
        # whether to override "header.frame_id" field to another frame,
        # or to take the value from subscribed messages
        self.odom_header_frame_id_override = self.get_parameter('odom_header_frame_id').value
        if self.odom_header_frame_id_override:
            self.odom_msg.header.frame_id = self.odom_header_frame_id_override
        # child_frame_id is always taken from parameter (defaults to base_link),
        # since the subscribed message lacks that information
        self.odom_msg.child_frame_id = self.get_parameter('odom_child_frame_id').value

        sub_topic = self.get_parameter('scanmatcher_pose_topic').value
        # Subscribe to the specified topic with PoseWithCovarianceStamped message
        # and call the callback function on each message
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            sub_topic,
            self.handle_pose_update,
            rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        )


    def handle_pose_update(self, msg):
        # copy timestamp directly from the subscribed message
        self.odom_msg.header.stamp = msg.header.stamp
        # if we are extrapolating pose with time, use the current timestamp instead
        # self.odom_msg.header.stamp = self.get_clock().now().to_msg()

        if not self.odom_header_frame_id_override:
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
