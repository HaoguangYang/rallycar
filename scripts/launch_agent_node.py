# ROS2 launch python api test
import rclpy
from rclpy.node import Node
import multiprocessing
import launch_ros
from launch import LaunchService
import nest_asyncio
import os
import signal
import sys
from std_msgs.msg import String

class Ros2LaunchListener(Node):
    def __init__(self):
        super().__init__("ros2_launch_agent_node")
        self.create_subscription(String, "activate_launch_file", self.start_launch_service, rclpy.qos....)
        self.create_subscription(String, "shutdown_launch_file", self.shutdown_launch_service, rclpy.qos....)
        self.processPool = {}

    def start_launch_service(self, msg):
        launch_file_dir = os.path.dirname(msg.data)
        launch_file = os.path.basename(msg.data)
        sys.path.append(launch_file_dir)
        # evil
        from eval(launch_file) import generate_launch_description as GLD
        ld = GLD()
        # Construct launch service pool. Each launch service can handle one launch description
        # add noninteractive=True to handle sigint
        lsPool = [LaunchService(noninteractive=True) for i in ld]
        i = 0
        for ls in lsPool:
            ls.include_launch_description(ld[i])
            i += 1
        # Construct process pool. Each process starts one launch service
        self.processPool.update{
            msg.data : [multiprocessing.Process(target=ls.run) for ls in lsPool]
        }
        for p in self.processPool[msg.data]:
            p.start()


    def shutdown_launch_service(self, msg):
        if msg.data not in self.processPool:
            self.get_logger().error("Unable to shutdown %s: Not started.", msg.data)
            return
        plist = self.processPool.pop(msg.data)
        for p in plist:
            # use SIGINT instead of SIGTERM to stop child processes ahead of launch service
            os.kill(p.pid, signal.SIGINT)
            # DON'T DO THIS as it will lead to a dirty shutdown with zombie nodes
            # p.terminate()
        for p in plist:
            p.join()
