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
        super().__init__('rad_vel_test_node')
        
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth = 1
        )

        # ----- 1. テスト用パラメータの設定 -----
        self.declare_parameter('test_pos_x', 5.0)  # 目標地点x
        self.declare_parameter('test_pos_y', 5.0)  # 目標地点y
        self.declare_parameter('max_speed', 0.5)   # 最大走行速度(m/s)

        # 角度PDゲイン（ステアリング制御用）
        self.e_n = 0.0
        self.e_n1 = 0.0
        self.k_p = 0.6
        self.k_d = 0.3

        # 速度PDゲイン（新規追加: 距離制御用）
        self.dist_e_n = 0.0
        self.dist_e_n1 = 0.0
        self.k_p_v = 0.8  # 距離に比例した速度定数
        self.k_d_v = 0.2  # 減速時のブレーキの強さ

        # 自己位置の初期化
        self.position_x = 0.0
        self.position_y = 0.0
        self.theta_z = 0.0

        # ----- 2. 通信設定 -----
        self.subscription = self.create_subscription(
            nav_msgs.Odometry, '/fusion/odom', self.get_odom, qos_profile)
        
        self.cmd_vel_publisher = self.create_publisher(
            geometry_msgs.Twist, 'cmd_vel', qos_profile)

        # 制御ループ (20Hz)
        self.timer = self.create_timer(0.05, self.robot_ctrl)

    def get_odom(self, msg):
        """自己位置の更新とクォータニオンからオイラー角への変換"""
        self.position_x = msg.pose.pose.position.x
        self.position_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        _, _, yaw = self.quaternion_to_euler(q.x, q.y, q.z, q.w)
        self.theta_z = yaw

    def sensim_angle(self, steering):
        """角度用のPD制御計算"""
        self.e_n = steering
        output = (self.k_p * self.e_n + self.k_d * (self.e_n - self.e_n1))
        self.e_n1 = self.e_n
        return output

    def sensim_speed(self, dist_error):
        """速度（距離）用のPD制御計算"""
        self.dist_e_n = dist_error
        # 距離の偏差に基づいた速度出力
        output = (self.k_p_v * self.dist_e_n + self.k_d_v * (self.dist_e_n - self.dist_e_n1))
        self.dist_e_n1 = self.dist_e_n
        return output

    def robot_ctrl(self):
        """車両制御メインループ"""
        tx = self.get_parameter('test_pos_x').value
        ty = self.get_parameter('test_pos_y').value
        limit_speed = self.get_parameter('max_speed').value

        # 1. 目標地点への相対位置と距離を計算
        rel_x = tx - self.position_x
        rel_y = ty - self.position_y
        dist = math.sqrt(rel_x**2 + rel_y**2)

        # 2. 角度制御の計算
        local_target_angle = math.atan2(rel_y, rel_x) - self.theta_z
        # 角度の正規化 (-pi ~ pi)
        local_target_angle = math.atan2(math.sin(local_target_angle), math.cos(local_target_angle))
        
        pd_angle_output = self.sensim_angle(local_target_angle)

        # 3. 速度制御の計算 (新規)
        # 目標距離 0 に対する PD 出力
        pd_speed_output = self.sensim_speed(dist)
        
        # 速度の上下限設定 (0.0 ～ limit_speed)
        target_linear_vel = max(0.0, min(limit_speed, pd_speed_output))

        # 4. 角速度の算出
        # 速度が落ちれば角速度も落ちるように設計
        target_angular_vel = target_linear_vel * pd_angle_output

        # 5. 停止判定 (目標地点から15cm以内なら完全に停止)
        if dist < 0.15:
            target_linear_vel = 0.0
            target_angular_vel = 0.0
            self.get_logger().info('Goal Reached!')

        # 6. メッセージの送信
        twist_msg = geometry_msgs.Twist()
        twist_msg.linear.x = target_linear_vel
        twist_msg.angular.z = target_angular_vel
        
        self.cmd_vel_publisher.publish(twist_msg)

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
