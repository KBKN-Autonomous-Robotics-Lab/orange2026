import rclpy
from rclpy.node import Node
import std_msgs.msg as std_msgs
import nav_msgs.msg as nav_msgs
import sensor_msgs.msg as sensor_msgs
import numpy as np
import math
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
import time
import geometry_msgs.msg as geometry_msgs
from rclpy.action import ActionServer
# my_msgsが環境にない場合はエラーになるため注意してください
# from my_msgs.action import StopFlag 
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Int32, String

class TestPF(Node):
    def __init__(self):
        super().__init__('path_follower_test_node')
        
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # ----- パラメータ設定 -----
        self.declare_parameter('twist_time', 60.0)      # 停止までの時間（秒）
        self.declare_parameter('linear_speed', 1.08)   # 前進速度
        self.declare_parameter('angular_speed', 0.5)   # 角速度

        self.limit_time = self.get_parameter('twist_time').get_parameter_value().double_value
        self.start_time = time.perf_counter() # 開始時間の記録
        self.running = True                   # 走行状態フラグ

        # Publisherを作成
        # モータードライバ(motor_driver_node)と合わせる場合は '/zlac8015d/twist/cmd_vel' に変更してください
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', qos_profile) 
        
        # タイマー設定 (0.05秒周期)
        self.timer = self.create_timer(0.05, self.robot_ctrl)
        self.get_logger().info(f'Test started: {self.limit_time}s remaining...')

    def check_finish_time(self):
        """経過時間をチェックし、制限時間を超えていればTrueを返す関数"""
        elapsed_time = time.perf_counter() - self.start_time
        return elapsed_time >= self.limit_time

    def robot_ctrl(self):
        # 終了判定
        if self.running and self.check_finish_time():
            self.stop_robot()
            return

        if not self.running:
            return

        # 走行用メッセージ作成
        speed_set = self.get_parameter('linear_speed').value
        angular_set = self.get_parameter('angular_speed').value
        
        twist_msg = Twist()
        twist_msg.linear.x = speed_set
        twist_msg.angular.z = angular_set
        
        self.cmd_vel_publisher.publish(twist_msg)

    def stop_robot(self):
        """ロボットを停止させ、ノードを終了する準備を行う"""
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(stop_msg)
        
        self.running = False
        self.get_logger().info('--- Time is up. Robot Stopped. ---')
        # 必要に応じてここで強制終了
        # rclpy.shutdown() 

def main(args=None):
    rclpy.init(args=args)
    path_follower_test = TestPF()
    try:
        rclpy.spin(path_follower_test)
    except KeyboardInterrupt:
        pass
    finally:
        path_follower_test.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
