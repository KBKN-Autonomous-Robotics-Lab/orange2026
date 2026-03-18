import rclpy
from rclpy.node import Node
import nav_msgs.msg as nav_msgs
import geometry_msgs.msg as geometry_msgs
import numpy as np
import math
import time
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

class PathFollowerTest(Node):
    def __init__(self):
        super().__init__('radius_comp_test_node')
        
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth = 1
        )

        # ----- 1. テスト用パラメータの設定 -----
        self.declare_parameter('test_pos_x', 5.0)  # 目標地点x
        self.declare_parameter('test_pos_y', 5.0)  # 目標地点y
        self.declare_parameter('target_speed', 0.5) # 走行速度(m/s)

        # PDゲインの初期化
        self.e_n = 0.0
        self.e_n1 = 0.0
        self.k_p = 0.6
        self.k_d = 0.3

        # 自己位置の初期化
        self.position_x = 0.0
        self.position_y = 0.0
        self.theta_z = 0.0

        # ----- 2. 通信設定 -----
        # 自己位置の購読
        self.subscription = self.create_subscription(
            nav_msgs.Odometry, '/fusion/odom', self.get_odom, qos_profile)
        
        # 速度命令のパブリッシャー
        self.cmd_vel_publisher = self.create_publisher(
            geometry_msgs.Twist, 'cmd_vel', qos_profile)

        # 制御ループ (20Hz)
        self.timer = self.create_timer(0.05, self.robot_ctrl)

    def get_odom(self, msg):
        """自己位置の更新とクォータニオンからオイラー角への変換"""
        self.position_x = msg.pose.pose.position.x
        self.position_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        # ユーティリティ関数を使用してYaw角を取得
        _, _, yaw = self.quaternion_to_euler(q.x, q.y, q.z, q.w)
        self.theta_z = yaw # 単位: radian

    def sensim0(self, steering):
        """PD制御の計算"""
        self.e_n = steering
        # 偏差の変化量に基づいたPD出力
        output = (self.k_p * self.e_n + self.k_d * (self.e_n - self.e_n1))
        self.e_n1 = self.e_n
        return output

    def robot_ctrl(self):
        """車両制御メインループ"""
        # パラメータの読み込み
        tx = self.get_parameter('test_pos_x').value
        ty = self.get_parameter('test_pos_y').value
        speed = self.get_parameter('target_speed').value

        # 1. 目標地点への相対位置を計算
        rel_x = tx - self.position_x
        rel_y = ty - self.position_y

        # 2. ロボット座標系への変換 (Yaw回転のみ考慮)
        # ロボットの正面に対する目標物の角度 e_n を算出
        local_target_angle = math.atan2(rel_y, rel_x) - self.theta_z
        
        # 角度を -pi ~ pi の範囲に正規化
        local_target_angle = math.atan2(math.sin(local_target_angle), math.cos(local_target_angle))

        # 3. PD制御の適用
        # 角度誤差から「曲率（どのくらい曲がるべきか）」を算出
        pd_output = self.sensim0(local_target_angle)

        # 4. 速度を考慮した角速度の導出
        # 提案式: angular.z = linear.x * PD出力
        target_angular_vel = speed * pd_output

        # 5. メッセージの作成と送信
        twist_msg = geometry_msgs.Twist()
        twist_msg.linear.x = speed
        twist_msg.angular.z = target_angular_vel
        
        self.cmd_vel_publisher.publish(twist_msg)

    # --- 不足していた既存のユーティリティ関数 ---
    def quaternion_to_euler(self, x, y, z, w):
        """クォータニオンからオイラー角への変換"""
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = math.asin(sinp) if abs(sinp) <= 1 else math.copysign(math.pi / 2, sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    node = PathFollowerTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
