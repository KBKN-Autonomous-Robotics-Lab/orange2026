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

        # ----- 1. パラメータの設定 -----
        self.declare_parameter('test_pos_x', 10.0)
        self.declare_parameter('test_pos_y', 10.0)
        self.declare_parameter('max_speed', 2.0)      # 最大速度 (m/s)
        self.declare_parameter('max_accel', 0.3)      # 加速度リミット (m/s^2)

        # 内部状態
        self.current_speed = 0.0      # 加速度制御用の現在の速度
        self.dt = 0.05                # 制御周期 (20Hz)
        self.goal_reached = False     # ゴール到達フラグ

        # PDゲイン
        self.e_n = 0.0
        self.e_n1 = 0.0
        self.k_p = 0.6
        self.k_d = 0.3

        # 自己位置
        self.position_x = 0.0
        self.position_y = 0.0
        self.theta_z = 0.0

        # ----- 2. 通信設定 -----
        self.subscription = self.create_subscription(
            nav_msgs.Odometry, '/odom', self.get_odom, qos_profile)
        
        self.cmd_vel_publisher = self.create_publisher(
            geometry_msgs.Twist, 'cmd_vel', qos_profile)

        # 制御ループ
        self.timer = self.create_timer(self.dt, self.robot_ctrl)

    def get_odom(self, msg):
        self.position_x = msg.pose.pose.position.x
        self.position_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = self.quaternion_to_euler(q.x, q.y, q.z, q.w)
        self.theta_z = yaw

    def sensim0(self, steering):
        self.e_n = steering
        output = (self.k_p * self.e_n + self.k_d * (self.e_n - self.e_n1))
        self.e_n1 = self.e_n
        return output

    def robot_ctrl(self):
        """車両制御メインループ（加速度制限 ＋ ゴール判定）"""
        
        # --- 0. ゴール到達後の処理 ---
        if self.goal_reached:
            self.stop_robot()
            return

        # パラメータ読み込み
        tx = self.get_parameter('test_pos_x').value
        ty = self.get_parameter('test_pos_y').value
        max_speed = self.get_parameter('max_speed').value
        max_accel = self.get_parameter('max_accel').value

        # --- 1. 距離計算とゴール判定 ---
        rel_x = tx - self.position_x
        rel_y = ty - self.position_y
        distance = math.sqrt(rel_x**2 + rel_y**2)

        if distance < 0.1:  # 10cm以内でゴール
            self.get_logger().info('Goal reached! Stopping...')
            self.goal_reached = True
            self.stop_robot()
            return

        # --- 2. 加速度リミッター処理 ---
        max_speed_change = max_accel * self.dt
        speed_diff = max_speed - self.current_speed

        if speed_diff > max_speed_change:
            self.current_speed += max_speed_change
        elif speed_diff < -max_speed_change:
            self.current_speed -= max_speed_change
        else:
            self.current_speed = max_speed

        # --- 3. 旋回制御 (PD) ---
        local_target_angle = math.atan2(rel_y, rel_x) - self.theta_z
        local_target_angle = math.atan2(math.sin(local_target_angle), math.cos(local_target_angle))
        
        pd_output = self.sensim0(local_target_angle)
        target_angular_vel = self.current_speed * pd_output
        # --- 4. 指令送信 ---
        twist_msg = geometry_msgs.Twist()
        twist_msg.linear.x = self.current_speed
        twist_msg.angular.z = target_angular_vel
        self.cmd_vel_publisher.publish(twist_msg)

    def stop_robot(self):
        """ロボットを完全に停止させる"""
        self.current_speed = 0.0
        msg = geometry_msgs.Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(msg)

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
