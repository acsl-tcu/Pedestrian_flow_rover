# メイン
import rclpy
from rclpy.node import Node

import os
import time
import math
import threading
import numpy as np
from datetime import datetime

# ROS2メッセージ型
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32


# ----------------- ROS2モジュールインポート -----------------
from ms_robot.yolo_sub import YOLO_SUB
from ms_robot.rover import ROVER
from ms_robot.rplidar_sub import RPLIDAR_SUB
from ms_robot.ricoh import RICOH
from ms_robot.tscf import TSCF
from ms_robot.cbf import CBF
from ms_robot.graph import save_animation, save_input_plot, save_distance_plot, save_trajectory_plot


# ----------------- 初期パラメータ設定 -----------------
def initialize_params():
    x0, y0 = 0.0, 2.0 # x0が負の値，y0が0.0の組み合わせはローバーが動かない ※(x0 = -3.0, y0 = 0.0)など
    params = {
        'ros': 1, # 0ならシミュレーション，1なら実機
        'vmax': 1.3, # ローバーの上限速度
        'vmin': 0.0, # ローバーの下限速度
        'wmax': np.pi / 2, # ローバーの上限角速度
        'wmin': -np.pi / 2, # ローバーの下限角速度
        'CBFr': 1.0, # CBFの半径
        'alpha': lambda h: 5 * h,
        'TSCF': {'f1':4.0, 'f2':1.0, 'k':2.0}, # {'f1': フィードバックゲイン, 'f2': フィードバックゲイン, 'k': 速度制御ゲイン}
        'A': np.diag([1, 1, 1]),
        'time': 0, # 時間
        'dt': 0.05, # 周期
        'px': x0, # 歩行者のx位置
        'py': y0, # 歩行者のy位置
        'ped_pos': np.array([[x0, y0]]), # 歩行者位置
        'goal': np.array([[np.nan, np.nan]]) # 目標位置
    }
    return params

# ----------------- 歩行者の動き(シミュレーション) -----------------
def update_pedestrian(px_list, py_list, rover_state, params,
                      ped_speed=0.8,
                      safe_dist=1.5,
                      fov_deg=120,
                      avoid_angle_deg=15,
                      max_yaw_rate_deg=15,
                      target_yaw_smooth=0.01):
    dt = params['dt']
    N = len(px_list)
    current_time = params['time']

    # 停止条件：3秒後から3秒間
    stop_start = 3.0
    stop_duration = 3.0
    stop_end = stop_start + stop_duration
    ped_stop_active = (stop_start <= current_time < stop_end)

    fov_half = np.deg2rad(fov_deg / 2)
    avoid_angle = np.deg2rad(avoid_angle_deg)
    max_yaw_rate = np.deg2rad(max_yaw_rate_deg)

    # 初期化
    if 'ped_yaw_all' not in params or 'ped_side_all' not in params:
        params['ped_yaw_all'] = [0.0] * N
        params['ped_side_all'] = [1] * N
        params['ped_initial_yaw_all'] = params['ped_yaw_all'].copy()

    ped_yaw_all = params['ped_yaw_all']
    ped_side_all = params['ped_side_all']
    ped_initial_yaw_all = params['ped_initial_yaw_all']

    ped_yaw_list = []
    ped_pos_list = []
    ped_vel_list = []

    for i in range(N):
        ped_yaw = ped_yaw_all[i]
        initial_yaw = ped_initial_yaw_all[i]

        # ローバーとの相対位置
        dx = rover_state[0] - px_list[i]
        dy = rover_state[1] - py_list[i]
        dist_to_rover = np.hypot(dx, dy)
        angle_to_rover = np.arctan2(dy, dx)

        # 歩行者正面基準での相対角度
        rel_angle = (angle_to_rover - ped_yaw + np.pi) % (2*np.pi) - np.pi
        in_fov = abs(rel_angle) <= fov_half

        # 回避行動
        if in_fov and dist_to_rover < safe_dist:
            side = -np.sign(rel_angle) if rel_angle != 0 else ped_side_all[i]
            ped_side_all[i] = side
            desired_target_yaw = ped_yaw + side * avoid_angle
            ped_yaw += target_yaw_smooth * (desired_target_yaw - ped_yaw)
        else:
            ped_yaw = initial_yaw

        # yawレート制限
        yaw_diff = (ped_yaw - ped_yaw_all[i] + np.pi) % (2*np.pi) - np.pi
        max_step = max_yaw_rate * dt
        yaw_step = np.clip(yaw_diff, -max_step, max_step)
        ped_yaw = ped_yaw_all[i] + yaw_step

        # 速度計算・位置更新
        if ped_stop_active:
            vx = 0.0
            vy = 0.0
        else:
            vx = ped_speed * np.cos(ped_yaw)
            vy = ped_speed * np.sin(ped_yaw)

        px_list[i] += vx * dt
        py_list[i] += vy * dt

        # 更新
        ped_yaw_all[i] = ped_yaw
        ped_yaw_list.append(ped_yaw)
        ped_pos_list.append([px_list[i], py_list[i]])
        ped_vel_list.append([vx, vy])

    # 時刻更新
    params['time'] += dt

    # paramsに反映
    params['ped_yaw_all'] = ped_yaw_all
    params['ped_side_all'] = ped_side_all

    # 配列化
    ped_pos_array = np.array(ped_pos_list)
    ped_vel_array = np.array(ped_vel_list)
    ped_yaw_array = np.array(ped_yaw_list)

    return ped_pos_array, ped_yaw_array, ped_vel_array

# ----------------- 行列B定義 -----------------
def B(theta):
    return np.array([
        [np.cos(theta), 0],
        [np.sin(theta), 0],
        [0, 1]
    ])


# ----------------- メイン実行 -----------------
def main():
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    print("✅ パスの設定が完了しました")

    save_select = 1 # 0ならデータ保存無し，1ならデータ保存有り
    params = initialize_params()
    print("✅ パラメータの設定が完了しました")

    x = np.zeros(3)
    u = np.zeros(2)
    idx = 0

    # ログ保存用
    x_log = []
    u_log = []
    goal_log = []
    ped_pos_all_log = []
    ped_vel_all_log = []
    ped_yaw_all_log = []
    print("✅ 変数の初期化が完了しました\n")

    if params['ros']:
        print("🔄 YOLO情報, LIDAR情報, ROVER情報を受信しています\n")
        rclpy.init()
        node = Node("robot_node")
        yolo = YOLO_SUB()
        lidar = RPLIDAR_SUB()
        rover = ROVER()
        ricoh = RICOH(yolo, lidar, rover)

        state_pub = node.create_publisher(Vector3, '/robot/state', 10)
        ped_pub = node.create_publisher(Vector3, '/pedestrian/state', 10)
        input_pub = node.create_publisher(Vector3, '/robot/input', 10)
        dist_pub = node.create_publisher(Float32, '/robot/pedestrian_distance', 10)

        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        executor.add_node(yolo)
        executor.add_node(lidar)
        executor.add_node(rover)
        executor.add_node(ricoh)

        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        print("\n🟡 実機モード\n")
    else:
        print("🟡 シミュレーションモード\n")

    txt = input("✅ プログラムを開始します [y/n]: ").lower()
    if txt=='y':
        time.sleep(2)
    elif txt=='n':
        print("❌ プログラムを中断します")
        return
    
    start_time = time.time()

    try:
        while (params['ros'] and params['time'] < 10 * 60) or (not params['ros'] and abs(x[0]) < 20 and params['time'] < 20):
            time.sleep(0.015)
            idx += 1

            if not params['ros']:
                # --- シミュレーション ---
                x = params['A'] @ x + B(x[2]) @ u * params['dt']
                x[2] = (x[2] + np.pi) % (2 * np.pi) - np.pi

                # --- 複数歩行者データの初期化 ---
                if 'ped_pos_all' not in params or not isinstance(params['ped_pos_all'], np.ndarray):
                    params['ped_pos_all'] = np.array([[params['px'], params['py']]])  # shape = (N,2) で初期化
                    params['ped_vel_all'] = np.zeros_like(params['ped_pos_all'])       # shape = (N,2)
                    params['ped_yaw_all'] = np.zeros(params['ped_pos_all'].shape[0])  # shape = (N,)

                # 歩行者位置と向きを更新
                ped_pos_all, ped_yaw_all, ped_vel_all = update_pedestrian(
                    params['ped_pos_all'][:,0],  # x座標リスト
                    params['ped_pos_all'][:,1],  # y座標リスト
                    x,                           # ローバー状態
                    params
                )

                # 更新結果を二次元配列として保存
                params['ped_pos_all'] = ped_pos_all        # shape = (N,2)
                params['ped_yaw_all'] = ped_yaw_all        # shape = (N,)
                params['ped_vel_all'] = ped_vel_all        # shape = (N,2)

                # 追従対象の歩行者を選択
                target_idx = params.get('target_idx', 0)
                target_idx = int(np.clip(target_idx, 0, ped_pos_all.shape[0]-1))

                # paramsに反映
                params['ped_pos'] = ped_pos_all[target_idx:target_idx+1, :]  # shape = (1,2)
                params['ped_vel'] = ped_vel_all[target_idx:target_idx+1, :]  # shape = (1,2)
                params['ped_yaw'] = ped_yaw_all[target_idx]                  # shape = ()
                params['ped_dis_all'] = np.linalg.norm(ped_pos_all - x[:2], axis=1)
                params['ped_dis'] = params['ped_dis_all'][target_idx]
            else:
                # 実機
                x = rover.x, rover.y, rover.yaw
                lidar.do()
                yolo.do()
                ricoh.do()

                # 歩行者位置の取得（RICOH）
                params['ped_pos_all'] = ricoh.info['pos_all']
                params['ped_vel_all'] = ricoh.info['vel_all']
                params['ped_yaw_all'] = ricoh.info['yaw_all']
                params['ped_dis_all'] = ricoh.info['dis_all']

                # 二次元配列として選択歩行者を格納
                params['ped_pos'] = ricoh.info['pos']
                params['ped_vel'] = ricoh.info['vel']
                params['ped_yaw'] = ricoh.info['yaw']
                params['ped_dis'] = ricoh.info['dis']

                # ローバーに制御入力を反映
                rover.input(u[0], u[1])

            # 時間軸状態制御形と制御バリア関数の適用
            u, params = TSCF(x, params)
            u, params = CBF(x, u, params)

            if params['ros']:
                # ロボット状態
                state_msg = Vector3()
                state_msg.x, state_msg.y, state_msg.z = float(x[0]), float(x[1]), float(x[2])
                state_pub.publish(state_msg)

                # 歩行者情報
                ped_msg = Vector3()
                ped_msg.x, ped_msg.y, ped_msg.z = float(params['ped_pos'][0,0]), float(params['ped_pos'][0,1]), float(params['CBFr'])
                ped_pub.publish(ped_msg)

                # 制御入力
                input_msg = Vector3()
                input_msg.x, input_msg.y = float(u[0]), float(u[1])
                input_pub.publish(input_msg)

                # 距離
                dist_msg = Float32()
                dist_msg.data = float(params['ped_dis'])
                dist_pub.publish(dist_msg)

            # --- ログ保存 ---
            x_log.append(np.array(x))
            u_log.append(u.copy())
            ped_pos_all_log.append(params['ped_pos_all'].copy())  # shape = (N,2)
            ped_vel_all_log.append(params['ped_vel_all'].copy())  # shape = (N,2)
            ped_yaw_all_log.append(params['ped_yaw_all'].copy())  # shape = (N,)
            goal = np.array(params['goal'], dtype=float).reshape(2,)
            goal_log.append(goal)

            print(f"target:[{params['ped_pos'][0,0]:.2f}][{params['ped_pos'][0,1]:.2f}] input:[{u[0]:.2f}][{u[1]:.2f}] "
                    f"state:[{x[0]:.2f}][{x[1]:.2f}][{x[2]:.2f}] dt:[{params['dt']*1000:.0f}ms] time:[{params['time']:.2f}]")

            # 経過時間更新
            current_time = time.time()
            params['dt'] = current_time - start_time
            start_time = current_time
            params['time'] += params['dt']

    except KeyboardInterrupt:
        print("\n❌ プログラムが中断されました")

    finally:
        if save_select:
            print("✅ プログラムを終了します\n")
            print("🔄 データを保存しています\n")
            save_animation(x_log, ped_pos_all_log, ped_yaw_all_log, goal_log, params)
            save_input_plot(u_log, params)
            save_distance_plot(ped_pos_all_log, x_log, params)
            save_trajectory_plot(x_log, ped_pos_all_log, ped_yaw_all_log, selected_id_index=0)
        else:
            print("✅ プログラムを終了します")
            print("❌ データを保存しません")

        if params['ros'] and rclpy.ok():
            # executor スレッド停止
            executor.shutdown()
            executor_thread.join()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
