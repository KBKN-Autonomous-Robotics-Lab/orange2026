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


class TestPF(Node):
    # コンストラクタです、PcdRotationクラスのインスタンスを作成する際に呼び出されます。
    def __init__(self):
        # 継承元のクラスを初期化します。
        super().__init__('path_follower_test_node')
        
        # 1. qos_profile の定義が必要です
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )
     # タイマーを0.05秒（50ミリ秒）ごとに呼び出す
        self.timer = self.create_timer(0.05, self.robot_ctrl)   
     # Publisherを作成
        self.cmd_vel_publisher = self.create_publisher(geometry_msgs.Twist, 'cmd_vel', qos_profile) #set publish pcd topic name##mosikasitara '/zlac8015d/twist/cmd_vel'ga topic ni narukamo
        
    def robot_ctrl(self):
         #set speed
        speed_set = 1.10#55 AutoNav 1.10
        
        #make msg
        twist_msg = geometry_msgs.Twist()
        twist_msg.linear.x = speed_set  # 前進速度 (m/s)
        twist_msg.angular.z = 1.0  # 角速度 (rad/s)
        
        self.cmd_vel_publisher.publish(twist_msg)
def main(args=None):
    # rclpyの初期化処理です。ノードを立ち上げる前に実装する必要があります。
    rclpy.init(args=args)
    # クラスのインスタンスを作成
    path_follower_test = TestPF()
    # spin処理を実行、spinをしていないとROS 2のノードはデータを入出力することが出来ません。
    rclpy.spin(path_follower_test)
    # 明示的にノードの終了処理を行います。
    path_follower_test.destroy_node()
    # rclpyの終了処理、これがないと適切にノードが破棄されないため様々な不具合が起こります。
    rclpy.shutdown()
# 本スクリプト(publish.py)の処理の開始地点です。
if __name__ == '__main__':
    # 関数`main`を実行する。
    main()
        
