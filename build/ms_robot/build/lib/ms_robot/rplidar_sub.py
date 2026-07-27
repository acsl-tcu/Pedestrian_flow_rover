# RPLIDARの点群を受信してフィルタ後の点群を /lidar_points に配信（過去点群バッファ対応版）
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import math
from collections import deque
import types
import time

class RPLIDAR_SUB(Node):
    """
    RPLIDAR情報受信ノード
    - /scan を購読
    - 点群フィルタ処理後に /lidar_points に配信
    - 過去の点群をタイムスタンプ付きでバッファに保持
    """

    def __init__(self, buffer_size=200):
        super().__init__("rplidar_sub_node")

        # 元の /scan を購読
        self.subscription = self.create_subscription(LaserScan, '/scan', self.subscriber_callback, 10)

        # フィルタ後の点群を PointCloud2 でパブリッシュ
        self.filtered_pub = self.create_publisher(PointCloud2, '/lidar_points', 10)

        # 現在のスキャンデータ
        self.ranges = []
        self.angles = []
        self.x = []
        self.y = []
        self.scan_msg = None
        self.scan_ready = False

        # 過去点群を保持するバッファ（タイムスタンプ付き）
        self.pc2_msg_buffer = deque(maxlen=buffer_size)

        self.wait_for_scan()

    def subscriber_callback(self, msg: LaserScan):
        """
        /scan 受信時に呼ばれるコールバック
        """
        self.scan_msg = msg
        self.scan_ready = True

    def wait_for_scan(self):
        while not self.scan_ready or self.scan_msg is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        print("✅ LIDAR情報を受信しました")

    def do(self):
        """
        最新スキャンデータを処理
        - 無効範囲除外
        - 不要角度除外
        - デカルト座標計算
        - PointCloud2 に変換して配信
        - バッファにタイムスタンプ付きで保存
        """
        if not self.scan_ready or self.scan_msg is None:
            self.get_logger().warn("❌ LIDAR情報を受信できませんでした")
            return

        # --- 元メッセージから距離・角度を抽出 ---
        angle_min = self.scan_msg.angle_min
        angle_increment = self.scan_msg.angle_increment
        ranges_full = np.array(self.scan_msg.ranges)

        # 無効な範囲を削除（0.1〜40m）
        valid = (ranges_full > 0.10) & (ranges_full < 40.0)
        ranges = ranges_full[valid]

        # 対応する角度を生成
        angles = angle_min + angle_increment * np.arange(len(ranges_full))
        angles = angles[valid]

        # --- 極座標 → デカルト変換 ---
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        # --- 不要な角度領域（柱など）を除外 ---
        mask = ~((angles > 10/12*math.pi) & (angles < math.pi) |
                 (angles < -10/12*math.pi) & (angles > -math.pi))

        self.ranges = ranges[mask]
        self.angles = angles[mask]
        self.x = x[mask]
        self.y = y[mask]

        # --- PointCloud2 に変換して配信 ---
        points = [(float(xi), float(yi), 0.0) for xi, yi in zip(self.x, self.y)]
        pc2_msg = pc2.create_cloud_xyz32(self.scan_msg.header, points)
        self.filtered_pub.publish(pc2_msg)

        # --- バッファに保存（タイムスタンプ付き） ---
        t = self.scan_msg.header.stamp.sec + self.scan_msg.header.stamp.nanosec * 1e-9
        lidar = types.SimpleNamespace()
        lidar.x = self.x
        lidar.y = self.y
        lidar.angles = self.angles
        lidar.ranges = self.ranges
        self.pc2_msg_buffer.append((t, lidar))

    def get_past_scan(self, target_time):
        if not self.pc2_msg_buffer:
            return None
        # 時刻差が最も小さいデータを探す
        times = np.array([t for t, _ in self.pc2_msg_buffer])
        idx = np.argmin(np.abs(times - target_time))
        return self.pc2_msg_buffer[idx][1]