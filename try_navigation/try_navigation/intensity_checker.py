#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np


class IntensityChecker(Node):
    def __init__(self):
        super().__init__('intensity_checker')

        # 地面点群を購読
        self.sub = self.create_subscription(
            PointCloud2,
            '/pcd_segment_ground',
            self.callback_ground,
            10
        )

        self.get_logger().info('intensity_checker started')
        self.get_logger().info('subscribe topic: /pcd_segment_ground')

    def callback_ground(self, msg):
        # PointCloud2 の data を float32 として読む
        # 今の環境では x, y, z, intensity の4つが float32 で入っている前提
        points = np.frombuffer(msg.data, dtype=np.float32)

        if len(points) == 0:
            self.get_logger().warn('No point data')
            return

        # x, y, z, intensity の4列に変換
        try:
            points = points.reshape(-1, 4)
        except ValueError:
            self.get_logger().error(
                f'PointCloud2 data cannot reshape to (-1, 4). data length = {len(points)}'
            )
            return

        intensity = points[:, 3]

        if len(intensity) == 0:
            self.get_logger().warn('No intensity data')
            return

        # 基本統計
        min_i = np.min(intensity)
        max_i = np.max(intensity)
        mean_i = np.mean(intensity)
        median_i = np.median(intensity)

        # パーセンタイル
        p50 = np.percentile(intensity, 50)
        p75 = np.percentile(intensity, 75)
        p90 = np.percentile(intensity, 90)
        p95 = np.percentile(intensity, 95)
        p98 = np.percentile(intensity, 98)
        p99 = np.percentile(intensity, 99)

        # とりあえずの推奨閾値
        # 白線だけを拾いたいので、95〜99%付近を候補にする
        recommended_low = p95
        recommended_high = p99

        self.get_logger().info(
            '\n'
            '========== Intensity Check ==========\n'
            f'points count : {len(intensity)}\n'
            f'min          : {min_i:.2f}\n'
            f'max          : {max_i:.2f}\n'
            f'mean         : {mean_i:.2f}\n'
            f'median       : {median_i:.2f}\n'
            f'50%          : {p50:.2f}\n'
            f'75%          : {p75:.2f}\n'
            f'90%          : {p90:.2f}\n'
            f'95%          : {p95:.2f}\n'
            f'98%          : {p98:.2f}\n'
            f'99%          : {p99:.2f}\n'
            f'recommended  : {recommended_low:.2f} ~ {recommended_high:.2f}\n'
            '====================================='
        )


def main(args=None):
    rclpy.init(args=args)
    node = IntensityChecker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
