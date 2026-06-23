# YOLO情報の受信
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from ms_yolo_msg.msg import StateYolo
from collections import deque
import time

class YOLO_SUB(Node):
    """
    YOLO情報受信ノード
    - /yolo_pose を購読
    - 最新メッセージ保持
    - 過去メッセージをタイムスタンプ付きでバッファに保持
    - kp_all を再構築（num x 17 x 2）
    """

    def __init__(self, buffer_size=200):
        super().__init__('yolo_sub_node')

        # メッセージから取得されるデータ（初期化）
        self.num = 0
        self.id = []
        self.cls = []
        self.deg_all = []
        self.degp_all = []
        self.degm_all = []
        self.kp_all = []           # 全員のキーポイント再構築
        self.selected_id = None
        self.selected_kp = []
        self.selected_deg = 0.0
        self.selected_degp = 0.0
        self.selected_degm = 0.0
        self.header = Header()

        # 過去メッセージを保持するバッファ
        self.yolo_msg_buffer = deque(maxlen=buffer_size)

        # サブスクライバ
        self.subscriber = self.create_subscription(
            StateYolo,
            '/yolo_pose',
            self.subscriber_callback,
            10
        )

        self.yolo_msg = None
        self.yolo_ready = False

        self.wait_for_yolo()

    def subscriber_callback(self, msg: StateYolo):
        self.yolo_msg = msg
        self.yolo_ready = True
        # 過去メッセージバッファに追加
        self.yolo_msg_buffer.append((time.time(), msg))

    def wait_for_yolo(self):
        while not self.yolo_ready or self.yolo_msg is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        print("✅ YOLO情報を受信しました")

    def do(self):
        """
        最新メッセージを内部変数に展開
        - flattenされたkp_allをnum x 17 x 2に復元
        """
        self.num = self.yolo_msg.num
        self.id = self.yolo_msg.id
        self.cls = self.yolo_msg.cls
        self.deg_all = getattr(self.yolo_msg, 'deg_all', [])
        self.degp_all = getattr(self.yolo_msg, 'degp_all', [])
        self.degm_all = getattr(self.yolo_msg, 'degm_all', [])

        # flattenされたkp_allを復元（17x2に分割）
        self.kp_all = []
        kp_flat = getattr(self.yolo_msg, 'kp_all', [])
        for i in range(self.num):
            start_idx = i * 17 * 2
            end_idx = start_idx + 17*2
            if end_idx <= len(kp_flat):
                person_kp = []
                for j in range(17):
                    x = kp_flat[start_idx + 2*j]
                    y = kp_flat[start_idx + 2*j + 1]
                    person_kp.append([x, y])
                self.kp_all.append(person_kp)

        # 選択ID情報
        self.selected_id = getattr(self.yolo_msg, 'selected_id', None)
        self.selected_kp = getattr(self.yolo_msg, 'selected_kp', [])
        self.selected_deg = getattr(self.yolo_msg, 'selected_deg', 0.0)
        self.selected_degp = getattr(self.yolo_msg, 'selected_degp', 0.0)
        self.selected_degm = getattr(self.yolo_msg, 'selected_degm', 0.0)
        # デバッグログ
        # self.get_logger().info(f"num={self.num}, ids={self.id}, selected_id={self.selected_id}")
        # self.get_logger().info(f"selected_kp={self.selected_kp}")
        # self.get_logger().info(f"deg_all={self.deg_all}")


def main(args=None):
    rclpy.init(args=args)
    node = YOLO_SUB()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.do()
            # 必要に応じてここでkp_allやselected_kpを使った処理
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
