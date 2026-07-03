# YOLO情報を送信（複数人対応＆選択ID抽出）
import cv2
from ultralytics import YOLO
from ms_yolo_msg.msg import StateYolo  # カスタムメッセージ
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
import time
import math
import threading

selected_id = 0
input_thread_started = False
USE_KEYPOINT = False  # Trueならキーポイント、FalseならBBox


class YOLO_PUB(Node):
    """ROS2ノードを定義するクラス"""
    def __init__(self):
        super().__init__('yolo_pub_node')
        self.pub_ = self.create_publisher(StateYolo, 'yolo_pose', 10)

    def pub(self, num, name, cls, ids_list,
            deg_all, degp_all, degm_all,
            kp_all, selected_kp,
            selected_deg, selected_degp, selected_degm,
            selected_id):
        """StateYoloメッセージを作成してPublish"""
        msg = StateYolo()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"

        msg.num = num
        msg.name = name
        msg.cls = cls
        msg.id = ids_list
        msg.deg_all = deg_all
        msg.degp_all = degp_all
        msg.degm_all = degm_all
        msg.kp_all = kp_all  # flatten済み

        # 選択ID情報
        msg.selected_kp = selected_kp
        msg.selected_deg = selected_deg
        msg.selected_degp = selected_degp
        msg.selected_degm = selected_degm
        msg.selected_id = float(selected_id)

        self.pub_.publish(msg)


def yolo(publisher):
    global selected_id, input_thread_started

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("❌ カメラの接続に失敗しました")
        exit()
    print("✅ カメラの接続に成功しました\n")
    print("✅ YOLOを開始します")
    if not input_thread_started:
        threading.Thread(target=input_thread_func, daemon=True).start()
        input_thread_started = True

    model = YOLO('yolo11x-pose.pt')
    conf = 0.6 # 信頼度の閾値
    max_det = 10 # 最大検出数

    # 以下の詳細やその他プロパティは公式サイト参照：[https://docs.ultralytics.com/ja/modes/predict/#image-and-video-formats]
    # source：推定を実行するファイル，0なら接続しているカメラのリアルタイムフレームになる
    # stream：すべてのフレームをメモリにロードする代わりに結果のジェネレーターを作成，ビデオやライブストリームを処理するのに有益
    # half：半精度(FP16)推論が可能になりサポートされているGPUでのモデル推論を精度への影響を最小限に抑えながら高速化することができる
    # results = model(source=0, conf=conf, stream=True, max_det=max_det, half=True)

    # trackモードで推論：追跡IDがつけられる，歩行者の判別が今後必要になったらこっちの方が有用かも
    # persist：現在の画像またはフレームがシーケンスの次であり現在の画像に前の画像からのトラックを期待することをトラッカーに伝える
    # 参考サイト：[https://docs.ultralytics.com/ja/modes/track/#why-choose-ultralytics-yolo-for-object-tracking]
    tracker = 'botsort.yaml' # 'botsort.yaml' or 'bytetrack.yaml'のどちらか，botsortが新しい方
    results = model.track(source=0, conf=conf, stream=True, max_det=max_det, half=True, persist=True, tracker=tracker)
    
    restrict_view = False
    offset_x = 0.0
    time_sta = time.time()

    while True:
        for r in results:
            # 時間計測
            time_end = time.time()
            dt = time_end - time_sta
            time_sta = time_end
            print(f"dt:{dt*1000:3.0f}ms ▶ 選択ID: {selected_id}")

            # 可視化
            plot = r.plot()
            h, w = plot.shape[:2]
            if restrict_view:
                view_w = int(w * 0.5)
                x_start = int((w - view_w) / 2 + offset_x * w)
                x_start = max(0, min(w - view_w, x_start))
                cropped = plot[0:h, x_start:x_start + view_w]
                display = cv2.resize(cropped, (w, h))
            else:
                display = plot

            # cv2.imshow("YOLO View", display) # カメラ映像を見るならコメントイン

            key = cv2.waitKey(1) & 0xFF
            if key == ord('s'):
                restrict_view = not restrict_view
                print("画角制限: {}".format("ON" if restrict_view else "OFF"))
            elif key == ord('q'):
                cap.release()
                cv2.destroyAllWindows()
                exit()
            elif restrict_view:
                if key == ord('a'):
                    offset_x -= 0.05
                elif key == ord('d'):
                    offset_x += 0.05
                offset_x = max(-0.5, min(0.5, offset_x))

            # 検出数
            num = len(r)
            if num <= 0:
                publisher.pub(0, [""], [0.0], [0.0], [], [], [], [], [], 0.0, 0.0, 0.0, selected_id)
                continue

            # IDs
            ids_list = [float(i.item()) for i in r.boxes.id] if r.boxes.id is not None else [0.0]*num

            # 全員のキーポイント・角度
            kp_all = []
            deg_all = []
            degp_all = []
            degm_all = []

            if USE_KEYPOINT:
                keypoints = r.keypoints.xyn.cpu().numpy()
                for person in keypoints:
                    kp_all.append(person.flatten().tolist())
                    left_shoulder = person[5]
                    right_shoulder = person[6]
                    if left_shoulder[0] < right_shoulder[0]:
                        xp_s = left_shoulder
                        xm_s = right_shoulder
                    else:
                        xp_s = right_shoulder
                        xm_s = left_shoulder
                    degm_all.append(2*math.pi*(-xm_s[0]+0.5))
                    degp_all.append(2*math.pi*(-xp_s[0]+0.5))
                    deg_all.append((degm_all[-1]+degp_all[-1])/2)
            else:
                keypoints = r.keypoints.xyn.cpu().numpy()
                for person in keypoints:
                    kp_all.append(person.flatten().tolist())
                boxes = r.boxes
                x = boxes.xywhn[:,0].cpu().numpy()
                xm = boxes.xyxyn[:,2].cpu().numpy()
                xp = boxes.xyxyn[:,0].cpu().numpy()
                deg_all = (2*math.pi*(-x+0.5)).tolist()
                degm_all = (2*math.pi*(-xm+0.5)).tolist()
                degp_all = (2*math.pi*(-xp+0.5)).tolist()

            # Flattenして1次元に
            kp_all_flat = [item for person in kp_all for item in person]

            # 選択IDの抽出
            if selected_id in ids_list:
                idx = ids_list.index(selected_id)
                kp_selected = kp_all[idx]
                deg_selected = deg_all[idx]
                degm_selected = degm_all[idx]
                degp_selected = degp_all[idx]
            else:
                kp_selected = []
                deg_selected = 0.0
                degm_selected = 0.0
                degp_selected = 0.0

            # Publish
            publisher.pub(num, ["" for _ in range(num)], [0.0 for _ in range(num)],
                          ids_list, deg_all, degp_all, degm_all,
                          kp_all_flat, kp_selected,
                          deg_selected, degp_selected, degm_selected,
                          selected_id)


def input_thread_func():
    global selected_id
    while True:
        user_input = input("\n🟡 選択IDを入力して \"Enter\": \n")
        if user_input.isdigit():
            selected_id = int(user_input)
            print(f"✅ 選択IDを {selected_id} に更新しました\n")
        else:
            print("❌ 数字を入力してください\n")


def main(args=None):
    rclpy.init(args=args)
    publisher = YOLO_PUB()
    yolo(publisher)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
