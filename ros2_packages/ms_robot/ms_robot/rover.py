# メガローバーに制御入力を送信し，オドメトリ情報を受信・配信（過去オドメトリバッファ対応版）
import rclpy
from rclpy.node import Node
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from math import sin, cos
import numpy as np
from collections import deque

# NumPy 1.20以降で削除された np.float 互換パッチ
if not hasattr(np, 'float'):
    np.float = float

def angle_normalize(a):
    """角度を -pi ~ pi に正規化"""
    return (a + np.pi) % (2 * np.pi) - np.pi

class ROVER(Node):
    """
    ローバー制御ノード
    - 制御入力送信
    - オドメトリ受信
    - 自己位置・姿勢の更新
    - 過去オドメトリをタイムスタンプ付きでバッファに保持
    """

    def __init__(self, buffer_size=200):
        super().__init__('rover_node')

        # 内部速度状態
        self.odo = {'v': 0.0, 'omega': 0.0}
        self.last_time = time.time()

        # 自己位置
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # QoS設定（Best Effort）
        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        # Publisher（ローバーへの速度コマンド）
        self.publisher = self.create_publisher(Twist, '/rover_twist', qos)

        # Subscriber（ローバーからのオドメトリ情報 = 速度）
        self.subscriber = self.create_subscription(Twist, '/rover_odo', self.subscriber_callback, qos)

        # Odometry publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # 過去オドメトリバッファ（タイムスタンプ付き）
        self.odom_msg_buffer = deque(maxlen=buffer_size)

        # Timerで周期的に自己位置更新
        self.create_timer(0.05, self.update_odom)

        # 初期入力（速度・角速度ゼロを送信）
        self.input(0.0, 0.0)
        self.last_odom_time = time.time()

        self.odo_msg = None
        self.rover_ready = False

        self.wait_for_rover()

    def subscriber_callback(self, msg: Twist):
        """
        /rover_odo 受信時に呼ばれるコールバック
        - 内部速度状態を更新
        """
        self.odo_msg = msg
        self.rover_ready = True
        self.odo['v'] = self.odo_msg.linear.x
        self.odo['omega'] = self.odo_msg.angular.z

    def wait_for_rover(self):
        while not self.rover_ready or self.odo_msg is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        print("✅ ROVER情報を受信しました")

    def input(self, v, omega):
        """
        ローバーに速度 v, 角速度 omega を送信する
        """
        dt = time.time() - self.last_time
        if dt > 0.02:  # 20msより短いとESP32でエラーになる
            self.odo_msg = Twist()
            self.odo_msg.linear.x = float(v)
            self.odo_msg.angular.z = float(omega)
            self.publisher.publish(self.odo_msg)
            self.last_time = time.time()

    def update_odom(self):
        """
        自己位置更新（オイラー積分）
        - Odometry メッセージを生成・配信
        - 過去オドメトリをバッファに保存
        """
        now = time.time()
        dt = now - self.last_odom_time
        if dt <= 0:
            return

        v = self.odo['v']
        omega = self.odo['omega']

        # オイラー積分で自己位置更新
        self.x += v * dt * cos(self.yaw)
        self.y += v * dt * sin(self.yaw)
        self.yaw += omega * dt

        # ここで -π～π に正規化
        self.yaw = angle_normalize(self.yaw)

        # Odometry メッセージ作成
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        # 四元数（yawのみ簡易計算）
        cy = np.cos(self.yaw * 0.5)
        sy = np.sin(self.yaw * 0.5)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = sy
        odom.pose.pose.orientation.w = cy

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega

        # 配信
        self.odom_pub.publish(odom)

        # --- バッファに保存（タイムスタンプ付き） ---
        self.odom_msg_buffer.append((now, [self.x, self.y, self.yaw]))

        # 時間更新
        self.last_odom_time = now

    def get_past_odom(self, target_time):
        """
        過去のオドメトリバッファから target_time に最も近い値を取得
        :param target_time: 取得したい時刻（秒）
        :return: [x, y, yaw] または None
        """
        if not self.odom_msg_buffer:
            return None
        return min(self.odom_msg_buffer, key=lambda x: abs(x[0] - target_time))[1]
    