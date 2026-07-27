import rclpy
from rclpy.node import Node
from dynamixel_sdk_custom_interfaces.msg import SetPosition
import time
import math

class SERVO(Node):
    def __init__(self, node_name="servo_publisher"):
        super().__init__(node_name)

        # パブリッシャの作成
        self.publisher_ = self.create_publisher(
            SetPosition,
            '/set_position',
            10  # QoS
        )
        self.msg = SetPosition()
        self.last_time = time.time()

        self.input(0.0)  # 初期指令（角度0）

    def input(self, theta):
        """
        サーボモータに角度コマンドを送信する（ラジアン指定: -π〜π）
        """
        current_time = time.time()
        if current_time - self.last_time > 0.01:  # 10ms以上経過
            self.msg.id = 1  # サーボID指定（必要に応じて変更）

            # 角度(rad)を0〜4000の範囲にスケーリング（-π → 4000, +π → 0）
            position = 2000 - theta / math.pi * 2000
            position = int(position)

            self.msg.position = position
            self.publisher_.publish(self.msg)
            self.get_logger().info(f"送信: ID={self.msg.id}, 位置={self.msg.position}")

            self.last_time = current_time
