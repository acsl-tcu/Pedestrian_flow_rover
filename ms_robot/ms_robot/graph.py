import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter
import os
from datetime import datetime

# ----------------- ヘルパー -----------------
def _ensure_xlog_array(x_log):
    if x_log is None or len(x_log) == 0:
        return None
    try:
        arrs = [np.ravel(x).astype(float) for x in x_log]
        x_arr = np.vstack(arrs)
    except Exception:
        x_arr = np.array(x_log, dtype=float)
        if x_arr.ndim == 1 and x_arr.size % 3 == 0:
            x_arr = x_arr.reshape(-1, 3)
    if x_arr.shape[1] != 3:
        raise ValueError("x_log must have 3 columns [x, y, theta].")
    return x_arr

def _ensure_ped_frame(ped):
    ped = np.array(ped, dtype=float)
    if ped.size == 0:
        return np.zeros((0,2), dtype=float)
    if ped.ndim == 1:
        return ped.reshape(1,2)
    return ped.reshape(-1,2)

def _ensure_yaw_frame(yaws):
    yaws = np.array(yaws, dtype=float)
    if yaws.size == 0:
        return np.array([], dtype=float)
    if yaws.ndim == 0:
        return np.array([float(yaws)])
    return yaws.reshape(-1)

# =========================================================
# ① アニメーション保存（誘導対象者のみ表示）
# =========================================================
def save_animation(x_log, ped_pos_all_log, ped_yaw_all_log,
                   goal_log, params, selected_id_index=0):

    x_arr = _ensure_xlog_array(x_log)
    ped_pos_all_log = np.array(ped_pos_all_log, dtype=object)
    ped_yaw_all_log = np.array(ped_yaw_all_log, dtype=object)
    goal_log = np.array(goal_log, dtype=float) if goal_log is not None else np.array([])

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlabel(r"$x$ [m]")
    ax.set_ylabel(r"$y$ [m]")
    ax.set_aspect('equal')
    ax.grid(True)

    # -------------------------------------------------
    # 描画範囲設定
    # -------------------------------------------------
    all_x, all_y = x_arr[:, 0].tolist(), x_arr[:, 1].tolist()
    for ped in ped_pos_all_log:
        p = _ensure_ped_frame(ped)
        if p.shape[0] > selected_id_index:
            all_x.append(p[selected_id_index, 0])
            all_y.append(p[selected_id_index, 1])

    x_min, x_max = min(all_x), max(all_x)
    y_min, y_max = min(all_y), max(all_y)
    r = max(x_max - x_min, y_max - y_min, 1.0)
    ax.set_xlim((x_min + x_max) / 2 - r / 2 - 1,
                (x_min + x_max) / 2 + r / 2 + 1)
    ax.set_ylim((y_min + y_max) / 2 - r / 2 - 1,
                (y_min + y_max) / 2 + r / 2 + 1)

    # -------------------------------------------------
    # ローバー（plotで統一）
    # -------------------------------------------------
    robot_point, = ax.plot([], [], 'bo', markersize=12, label='Vehicle')
    robot_traj,  = ax.plot([], [], 'b--')
    robot_arrow  = ax.quiver([], [], [], [], color='blue',
                             scale=6, width=0.01)

    # -------------------------------------------------
    # 歩行者（ローバーと完全に同じ描画方式）
    # -------------------------------------------------
    ped_point, = ax.plot([], [], 'ro', markersize=12, label='Pedestrian')
    ped_traj,  = ax.plot([], [], 'r--')
    ped_arrow  = None

    # -------------------------------------------------
    # CBF
    # -------------------------------------------------
    cbf_circle = plt.Circle((0, 0), params.get('CBFr', 1.0),
                            color='green', alpha=0.3, label='CBF')
    ax.add_patch(cbf_circle)

    # -------------------------------------------------
    # ゴール
    # -------------------------------------------------
    goal_point, = ax.plot([], [], 'g*', markersize=12, label='Goal')

    ax.legend()

    # -------------------------------------------------
    # 更新関数
    # -------------------------------------------------
    def update(frame):
        nonlocal ped_arrow

        # ---------- ローバー ----------
        robot_point.set_data([x_arr[frame, 0]], [x_arr[frame, 1]])
        robot_traj.set_data(x_arr[:frame + 1, 0], x_arr[:frame + 1, 1])
        robot_arrow.set_offsets([[x_arr[frame, 0], x_arr[frame, 1]]])
        robot_arrow.set_UVC([np.cos(x_arr[frame, 2])],
                            [np.sin(x_arr[frame, 2])])

        # ---------- ゴール ----------
        if frame < len(goal_log):
            goal_point.set_data([goal_log[frame, 0]], [goal_log[frame, 1]])
        else:
            goal_point.set_data([], [])

        # ---------- 歩行者 ----------
        if frame < len(ped_pos_all_log):
            p = _ensure_ped_frame(ped_pos_all_log[frame])
            if p.shape[0] > selected_id_index:
                ped_xy = p[selected_id_index]

                ped_point.set_data([ped_xy[0]], [ped_xy[1]])
                cbf_circle.center = (ped_xy[0], ped_xy[1])

                tx, ty = [], []
                for t in range(frame + 1):
                    pt = _ensure_ped_frame(ped_pos_all_log[t])
                    if pt.shape[0] > selected_id_index:
                        tx.append(pt[selected_id_index, 0])
                        ty.append(pt[selected_id_index, 1])
                ped_traj.set_data(tx, ty)

                if ped_arrow is not None:
                    ped_arrow.remove()

                yaws = _ensure_yaw_frame(ped_yaw_all_log[frame])
                if len(yaws) > selected_id_index:
                    ped_arrow = ax.quiver(
                        ped_xy[0], ped_xy[1],
                        np.cos(yaws[selected_id_index]),
                        np.sin(yaws[selected_id_index]),
                        color='red', scale=6, width=0.01
                    )

        return []

    # -------------------------------------------------
    # アニメーション保存
    # -------------------------------------------------
    anim = FuncAnimation(fig, update,
                         frames=len(x_arr), interval=15)

    save_dir = os.path.expanduser("~/MovingSignage_data/Video")
    os.makedirs(save_dir, exist_ok=True)
    now_str = datetime.now().strftime("%Y_%m.%d_%H-%M")
    save_path = os.path.join(save_dir, f"{now_str}.mp4")

    writer = FFMpegWriter(fps=30, bitrate=1800)
    anim.save(save_path, writer=writer)
    print(f"✅ 動画を保存しました: {save_path}")

    plt.show()

# =========================================================
# ② 入力グラフ（角速度：赤，凡例：Pedestrian）
# =========================================================
def save_input_plot(u_log, params):
    u_log = np.array(u_log)
    time_axis = np.arange(len(u_log)) * params['dt']

    plt.figure()

    # 並進速度（青）
    plt.plot(time_axis, u_log[:,0], 'b', label='Velocity')

    # 角速度（赤）
    plt.plot(time_axis, u_log[:,1], 'r', label='Angular Velocity')

    plt.xlabel(r"$t$ [s]")
    plt.ylabel(r"$v$ [m/s], $ω$ [rad/s]")
    plt.legend()
    plt.grid()
    plt.gca().set_box_aspect(1)

    save_dir = os.path.expanduser("~/MovingSignage_data/Input")
    os.makedirs(save_dir, exist_ok=True)
    now_str = datetime.now().strftime("%Y_%m.%d_%H-%M")
    save_path = os.path.join(save_dir, f"input_{now_str}.png")

    plt.savefig(save_path)
    print(f"✅ 入力グラフを保存しました: {save_path}")
    plt.show()

# =========================================================
# ③ 距離グラフ（誘導対象者のみ）
# =========================================================
def save_distance_plot(ped_pos_all_log, x_log, params, selected_id_index=0):
    x_arr = _ensure_xlog_array(x_log)
    ped_pos_all_log = np.array(ped_pos_all_log, dtype=object)

    time_axis = np.arange(len(x_arr)) * params['dt']
    dists = []

    for i in range(len(x_arr)):
        p = _ensure_ped_frame(ped_pos_all_log[i])
        if p.shape[0] > selected_id_index:
            d = np.linalg.norm(p[selected_id_index] - x_arr[i,:2])
            dists.append(d)
        else:
            dists.append(np.nan)

    plt.figure()
    plt.plot(time_axis, dists, label='Pedestrian')
    plt.axhline(params.get('CBFr',1.0), linestyle='--', color='red', label='CBF')
    plt.xlabel(r"$t$ [s]")
    plt.ylabel(r"$d$ [m]")
    plt.legend()
    plt.grid()
    plt.gca().set_box_aspect(1)

    save_dir = os.path.expanduser("~/MovingSignage_data/Distance")
    os.makedirs(save_dir, exist_ok=True)
    now_str = datetime.now().strftime("%Y_%m.%d_%H-%M")
    save_path = os.path.join(save_dir, f"distance_{now_str}.png")
    plt.savefig(save_path)
    print(f"✅ 距離グラフを保存しました: {save_path}")
    plt.show()

# =========================================================
# ④ 軌跡グラフ（誘導対象者のみ）
# =========================================================
def save_trajectory_plot(x_log, ped_pos_all_log, ped_yaw_all_log, selected_id_index=0):

    x_arr = _ensure_xlog_array(x_log)
    ped_pos_all_log = np.array(ped_pos_all_log, dtype=object)
    ped_yaw_all_log = np.array(ped_yaw_all_log, dtype=object)

    plt.figure(figsize=(8,8))
    plt.plot(x_arr[:,0], x_arr[:,1], 'b-', label='Vehicle')

    traj = []
    for ped in ped_pos_all_log:
        p = _ensure_ped_frame(ped)
        if p.shape[0] > selected_id_index:
            traj.append(p[selected_id_index])
    traj = np.array(traj)

    plt.plot(traj[:,0], traj[:,1], 'r-', label='Pedestrian')

    step = max(1, len(x_arr)//20)
    for i in range(0, len(x_arr), step):
        plt.arrow(
            x_arr[i,0], x_arr[i,1],
            0.3*np.cos(x_arr[i,2]),
            0.3*np.sin(x_arr[i,2]),
            head_width=0.1, color='blue'
        )
        if i < len(ped_yaw_all_log):
            y = _ensure_yaw_frame(ped_yaw_all_log[i])
            if len(y) > selected_id_index:
                plt.arrow(
                    traj[i,0], traj[i,1],
                    0.3*np.cos(y[selected_id_index]),
                    0.3*np.sin(y[selected_id_index]),
                    head_width=0.1, color='red'
                )

    plt.xlabel(r"$x$ [m]")
    plt.ylabel(r"$y$ [m]")
    plt.legend()
    plt.grid()
    plt.gca().set_box_aspect(1)

    save_dir = os.path.expanduser("~/MovingSignage_data/Trajectory")
    os.makedirs(save_dir, exist_ok=True)
    now_str = datetime.now().strftime("%Y_%m.%d_%H-%M")
    save_path = os.path.join(save_dir, f"trajectory_{now_str}.png")
    plt.savefig(save_path)
    print(f"✅ 軌跡グラフを保存しました: {save_path}")
    plt.show()
