#!/usr/bin/env python3

# The serial port driver that interfaces with the RallyCar.
# Receives /steering_cmd and /accelerator_cmd, and writes to serial port at 50Hz.
# Creates services that turns on/off IMU measurements.

import rospy
from geometry_msgs.msg import Point32
from std_msgs.msg import Time, ColorRGBA
from sensor_msgs.msg import Imu


class ImuParser(object):
    def __init__(self):
        self.imu_data = Imu()
        self.imu_data.header.frame_id = 'imu_frame'
        rospy.init_node("imu_parser")
        self.imu_pub = rospy.Publisher("imu", Imu, queue_size=1)
        rospy.Subscriber("imu/accelerometer", Point32, self.recv_accel)
        rospy.Subscriber("imu/gyroscope", Point32, self.recv_gyro)
        rospy.Subscriber("imu/orientation", ColorRGBA, self.recv_orientation)
        rospy.Subscriber("imu/stamp", Time, self.recv_timestamp)

    def recv_accel(self, msg):
        self.imu_data.linear_acceleration.x, \
            self.imu_data.linear_acceleration.y, \
                self.imu_data.linear_acceleration.z,= (msg.x, msg.y, msg.z)

    def recv_gyro(self, msg):
        self.imu_data.angular_velocity.x, \
            self.imu_data.angular_velocity.y, \
                self.imu_data.angular_velocity.z,= (msg.x, msg.y, msg.z)

    def recv_orientation(self, msg):
        self.imu_data.orientation.x, \
            self.imu_data.orientation.y, \
                self.imu_data.orientation.z, \
                    self.imu_data.orientation.w = (msg.r, msg.g, msg.b, msg.a)

    def recv_timestamp(self, msg):
        self.imu_data.header.stamp = msg.data
        self.imu_pub.publish(self.imu_data)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    ImuParser().run()
