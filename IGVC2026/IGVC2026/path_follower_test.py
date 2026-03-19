# rclpy (ROS 2のpythonクライアント)の機能を使えるようにします。
import rclpy
# rclpy (ROS 2のpythonクライアント)の機能のうちNodeを簡単に使えるようにします。こう書いていない場合、Nodeではなくrclpy.node.Nodeと書く必要があります。
from rclpy.node import Node
import std_msgs.msg as std_msgs
import nav_msgs.msg as nav_msgs
import sensor_msgs.msg as sensor_msgs
import numpy as np
import math
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
import time
import geometry_msgs.msg as geometry_msgs
from rclpy.action import ActionServer ####
from my_msgs.action import StopFlag ####
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from std_msgs.msg import String
import threading
from std_msgs.msg import Bool
from rclpy.executors import MultiThreadedExecutor


class StopFlagActionServer(Node):
    def __init__(self):
        super().__init__('stop_flag_action_server')
        self.white_line_detected = False
        self.stop_sign_mode = False
        self.create_subscription(
            Bool,
            '/simulate_white_line',
            self.white_line_callback,
            10
            )
        self.server = ActionServer(self, StopFlag, "stop_flag", execute_callback=self.listener_callback)

    def white_line_callback(self, msg):
        if self.stop_sign_mode:
            self.white_line_detected = msg.data
            if msg.data:
                self.get_logger().info("White line simulated via topic")




    def listener_callback(self, goal_handle):
        a = goal_handle.request.a
        b = goal_handle.request.b
        self.get_logger().info(f"Received goal with a: {a}, b: {b}")

        result = StopFlag.Result()

        # -------------------------
        # stop sign goal
        # -------------------------
        if a == 2:
            self.stop_sign_mode = True
            self.get_logger().info("Stop sign mode start")
            
            # stop sign goal は常に処理
            # ここで白線認識し、停止信号を出す予定。
            while rclpy.ok():
                if self.white_line_detected:
                    self.get_logger().info("white line detected")
                    result.sum = 999
                    goal_handle.succeed()
                    self.white_line_detected = False
                    self.stop_sign_mode = False
                    self.get_logger().info("waiting for white line")
                    return result
                
                    

                else:
                    self.get_logger().info("white line not detected")
                
            #result.sum = 999
                time.sleep(0.2)

        else:
            self.get_logger().info("Human or Tire goal received")
            result.sum = a + b
            goal_handle.succeed()

        return result

def main(args=None):
    rclpy.init(args=args)
    node = StopFlagActionServer()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    executor.spin()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

