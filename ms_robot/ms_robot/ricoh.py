# 歩行者位置推定

import rclpy
from rclpy.node import Node
import numpy as np
import math
import time
from ms_yolo_msg.msg import StateYolo
from std_msgs.msg import Float32MultiArray


def angle_normalize(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


class KF:
    """
    状態: [x, y, vx, vy]
    観測: [x, y]
    yaw = atan2(vy, vx)（派生量）
    """
    def __init__(self, x, y):
        self.x = np.array([x, y, 0.0, 0.0], dtype=float)
        self.P = np.eye(4) * 1.0
        self.R = np.diag([0.1, 0.1])

    def predict(self, dt):
        if dt <= 0:
            return

        F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1,  0],
            [0, 0, 0,  1]
        ])

        sigma_a2 = 1.0  # (m/s^2)^2
        Q = np.array([
            [dt**4/4, 0,        dt**3/2, 0       ],
            [0,       dt**4/4,  0,       dt**3/2 ],
            [dt**3/2, 0,        dt**2,    0       ],
            [0,       dt**3/2,  0,        dt**2   ]
        ]) * sigma_a2

        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def update(self, z):
        z = np.array(z, dtype=float)

        H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])

        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        y = z - H @ self.x
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ H) @ self.P

    # ---- 出力 ----
    def pos(self):
        return self.x[0], self.x[1]

    def vel_raw(self):
        return self.x[2], self.x[3]

    def yaw(self):
        vx, vy = self.vel_raw()
        if math.hypot(vx, vy) < 0.1:
            return np.nan
        return math.atan2(vy, vx)


class RICOH(Node):
    def __init__(self, yolo_sub=None, lidar_sub=None, rover=None):
        super().__init__('ricoh_node_delay')

        self.yolo_sub = yolo_sub
        self.lidar_sub = lidar_sub
        self.rover = rover

        self.info_pub = self.create_publisher(
            Float32MultiArray, '/pedestrian_info', 10
        )

        self.trackers = {}

        self.info = {
            'pos_all': np.empty((0, 2)),
            'vel_all': np.empty((0, 2)),
            'yaw_all': np.empty((0,)),
            'dis_all': np.empty((0,)),
            'pos': np.empty((1, 2)),
            'vel': np.empty((1, 2)),
            'yaw': np.nan,
            'dis': np.nan,
            'index': -1,
            'confirmed': np.empty((0, 2))
        }

        self.GATE_THRESHOLD = 0.6
        self.prev_time = time.time()

    def do(self, delay=0.25):
        if self.yolo_sub is None or self.lidar_sub is None or self.rover is None:
            return

        yolo = self.yolo_sub.yolo_msg
        lidar = self.lidar_sub.get_past_scan(time.time() - delay)
        vehicle_pose = self.rover.get_past_odom(time.time() - delay)

        if lidar is None or vehicle_pose is None or yolo is None:
            return

        now = time.time()
        dt = max(1e-6, now - self.prev_time)

        # -------- 測定生成 --------
        degm_all = np.array(getattr(yolo, 'degm_all', []))
        degp_all = np.array(getattr(yolo, 'degp_all', []))
        deg_all = np.array(getattr(yolo, 'deg_all', []))
        ids = np.array(yolo.id)
        num = min(int(yolo.num), len(ids))
        selected_id = getattr(yolo, 'selected_id', -1)

        meas_positions = {}
        for i in range(num):
            mask = (lidar.angles > degm_all[i]) & (lidar.angles < degp_all[i])
            tempx, tempy = lidar.x[mask], lidar.y[mask]
            valid = np.isfinite(tempx) & np.isfinite(tempy)
            tempx, tempy = tempx[valid], tempy[valid]
            if tempx.size == 0:
                continue

            dis = float(np.min(np.sqrt(tempx**2 + tempy**2)))
            ang = angle_normalize(vehicle_pose[2] + deg_all[i])
            x = dis * math.cos(ang) + vehicle_pose[0]
            y = dis * math.sin(ang) + vehicle_pose[1]

            meas_positions[ids[i]] = (x, y, dis)

        # -------- 予測 & timeout --------
        for tid in list(self.trackers.keys()):
            tr = self.trackers[tid]
            tr['kf'].predict(dt)
            if now - tr['last_seen'] > 2.0:
                del self.trackers[tid]

        # -------- 更新 --------
        for tid, (mx, my, _) in meas_positions.items():
            if tid not in self.trackers:
                self.trackers[tid] = {
                    'kf': KF(mx, my),
                    'last_seen': now
                }
                continue

            tr = self.trackers[tid]
            px, py = tr['kf'].pos()

            if np.linalg.norm([mx - px, my - py]) <= self.GATE_THRESHOLD:
                tr['kf'].update([mx, my])
                tr['last_seen'] = now

        # -------- 出力 --------
        out_ids = [i for i in ids if i in self.trackers]

        pos_all, vel_all, yaw_all, dist_all = [], [], [], []

        for tid in out_ids:
            tr = self.trackers[tid]
            px, py = tr['kf'].pos()
            vx, vy = tr['kf'].vel_raw()
            yaw = tr['kf'].yaw()

            pos_all.append([px, py])
            vel_all.append([vx, vy])
            yaw_all.append(yaw)
            dist_all.append(
                math.hypot(px - vehicle_pose[0], py - vehicle_pose[1])
            )

        self.info['pos_all'] = np.array(pos_all)
        self.info['vel_all'] = np.array(vel_all)
        self.info['yaw_all'] = np.array(yaw_all)
        self.info['dis_all'] = np.array(dist_all)
        self.info['confirmed'] = self.info['pos_all'].copy()

        if selected_id != 0 and selected_id in out_ids:
            # ① 明示的にIDが指定されている場合のみID優先
            idx = out_ids.index(selected_id)
        elif len(dist_all) > 0:
            # ② selected_id == 0 → 最も近い人
            idx = int(np.argmin(dist_all))
        else:
            idx = -1

        self.info['index'] = idx
        if idx >= 0:
            self.info['pos'] = np.array([pos_all[idx]])
            self.info['vel'] = np.array([vel_all[idx]])
            self.info['yaw'] = yaw_all[idx]
            self.info['dis'] = dist_all[idx]
        else:
            self.info['pos'] = np.array([[np.nan, np.nan]])
            self.info['vel'] = np.array([[0.0, 0.0]])
            self.info['yaw'] = np.nan
            self.info['dis'] = np.nan

        self.prev_time = now

        if idx >= 0:
            msg = Float32MultiArray()
            msg.data = [
                float(self.info['dis']),
                float(deg_all[idx]),
                float(self.info['yaw']),
                float(deg_all[idx] + self.info['yaw'])
            ]
            self.info_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RICOH()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
