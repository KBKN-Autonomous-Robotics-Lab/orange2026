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
        super().__init__('rad_vel_ac_test_node')
        
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth = 1
        )

        # ----- 1. テスト用パラメータの設定 -----
        self.declare_parameter('test_pos_x', 10.0)
        self.declare_parameter('test_pos_y', 10.0)
        self.declare_parameter('max_speed', 1.5)

        # 角度PDゲイン
        self.e_n = 0.0
        self.e_n1 = 0.0
        self.k_p = 0.6
        self.k_d = 0.3

        # 速度PDゲイン
        self.dist_e_n = 0.0
        self.dist_e_n1 = 0.0
        self.k_p_v = 0.2
        self.k_d_v = 0.1

        # 加速度リミッタ用の変数（新規追加）
        self.prev_linear_vel = 0.0  # 前回のループでの速度
        self.accel_limit = 0.04     # 1ループ(0.05s)あたりの速度変化の最大値

        # 自己位置の初期化
        self.position_x = 0.0
        self.position_y = 0.0
        self.theta_z = 0.0

        # ----- 2. 通信設定 -----
        self.subscription = self.create_subscription(
            nav_msgs.Odometry, '/odom', self.get_odom, qos_profile)
        
        self.cmd_vel_publisher = self.create_publisher(
            geometry_msgs.Twist, 'cmd_vel', qos_profile)

        self.timer = self.create_timer(0.05, self.robot_ctrl)

    def get_odom(self, msg):
        self.position_x = msg.pose.pose.position.x
        self.position_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = self.quaternion_to_euler(q.x, q.y, q.z, q.w)
        self.theta_z = yaw

    def sensim_angle(self, steering):
        self.e_n = steering
        output = (self.k_p * self.e_n + self.k_d * (self.e_n - self.e_n1))
        self.e_n1 = self.e_n
        return output

    def sensim_speed(self, dist_error):
        self.dist_e_n = dist_error
        output = (self.k_p_v * self.dist_e_n + self.k_d_v * (self.dist_e_n - self.dist_e_n1))
        self.dist_e_n1 = self.dist_e_n
        return output

    def robot_ctrl(self):
        tx = self.get_parameter('test_pos_x').value
        ty = self.get_parameter('test_pos_y').value
        limit_speed = self.get_parameter('max_speed').value

        rel_x = tx - self.position_x
        rel_y = ty - self.position_y
        dist = math.sqrt(rel_x**2 + rel_y**2)

        local_target_angle = math.atan2(rel_y, rel_x) - self.theta_z
        local_target_angle = math.atan2(math.sin(local_target_angle), math.cos(local_target_angle))
        pd_angle_output = self.sensim_angle(local_target_angle)

        # 速度PD計算
        pd_speed_output = self.sensim_speed(dist)
        # PDによる目標速度（理想値）
        raw_target_vel = max(0.0, min(limit_speed, pd_speed_output))

        # --- 加速度リミッタの適用 ---
        vel_diff = raw_target_vel - self.prev_linear_vel
        if abs(vel_diff) > self.accel_limit:
            target_linear_vel = self.prev_linear_vel + math.copysign(self.accel_limit, vel_diff)
        else:
            target_linear_vel = raw_target_vel
        
        self.prev_linear_vel = target_linear_vel
        # -------------------------

        target_angular_vel = target_linear_vel * pd_angle_output

        if dist < 0.15:
            target_linear_vel = 0.0
            target_angular_vel = 0.0
            self.prev_linear_vel = 0.0 # 停止時は履歴もリセット
            self.get_logger().info('Goal Reached!')

        twist_msg = geometry_msgs.Twist()
        twist_msg.linear.x = target_linear_vel
        twist_msg.angular.z = target_angular_vel
        self.cmd_vel_publisher.publish(twist_msg)

    def quaternion_to_euler(self, x, y, z, w):
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
