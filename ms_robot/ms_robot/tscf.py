import numpy as np

def calcZ(goal, x):
    # 一応zを計算したけど角速度が振動するので使わない
    # goal_dx = goal[0] - x[0]
    # goal_dy = goal[1] - x[1]
    # goal_thetaD = np.arctan2(goal_dy, goal_dx)
    # z = -goal_dx * np.sin(goal_thetaD) + goal_dy * np.cos(goal_thetaD)
    z = 0
    return z

def calcKr(ped_vel, prev_ped_yaw, ped_yaw, dt):
    # 一応Krを計算したけど角速度が振動するので使わない
    # if np.linalg.norm(ped_vel) < 1e-3:
    #     return 0.0, prev_ped_yaw
    # Kr = (ped_yaw - prev_ped_yaw) / (np.linalg.norm(ped_vel) * dt)
    Kr = 0
    return Kr, ped_yaw

def TSCF(x, params):
    """
    TSCF
    x : 状態ベクトル [x, y, theta]
    params : 辞書型パラメータ
        params['CBF'] = [歩行者x, 歩行者y, 半径]
        params['TSCF'] = {'kp': 比例ゲイン, 'f1': フィードバックゲイン, 'f2': フィードバックゲイン}
        params['dis'] = 目標までの距離（通常は歩行者距離）
        params['vmax'] = 最大速度
        params['wmax'] = 最大角速度
        params['theta_offset_deg'] = 円周上目標角度（歩行者基準、任意）
        params['use_center_as_goal'] = True/False（歩行者中心を目標にするか）
    戻り値:
        u: 入力ベクトル [v, omega]
        params: 更新済みパラメータ
    """

    u = np.zeros(2)  # 初期化

    if np.isnan(params['ped_pos']).any():
        u = np.zeros(2)
        return u, params

    # --- 歩行者情報 ---
    ped_x = params['ped_pos'][0, 0]
    ped_y = params['ped_pos'][0, 1]
    ped_yaw = params['ped_yaw']
    CBFr = params['CBFr']

    # --- 目標位置設定 ---
    goal_r = CBFr + 0.3
    goal_theta = ped_yaw - np.pi / 8

    # --- 歩行者中心を目標にするか円周上目標にするか ---
    use_center_as_goal = params.get('use_center_as_goal', False)
    if use_center_as_goal:
        goal = np.array([ped_x, ped_y])
    else:
        # 円周上の目標位置
        goal = np.array([ped_x + goal_r * np.cos(goal_theta),
                         ped_y + goal_r * np.sin(goal_theta)])
    params['goal'] = goal

    # --- 目標位置との距離と角度差 ---
    thetaD = np.arctan2(goal[1] - x[1], goal[0] - x[0]) - x[2]  # 角度差
    goal_dis = np.linalg.norm(goal - x[:2])

    # ±πの範囲に補正
    thetaD = (thetaD + np.pi) % (2*np.pi) - np.pi

    # 角度差を±90度に制限したい場合は以下を使う
    # thetaD = (thetaD + np.pi / 2) % np.pi - np.pi / 2

    # --- TSCF 制御計算 ---
    z = calcZ(params['goal'], x)

    if 'prev_ped_yaw' not in params:
        params['prev_ped_yaw'] = 0.0
    kr, ped_yaw = calcKr(params['ped_vel'], params['prev_ped_yaw'], params['ped_yaw'], params['dt'])
    params['prev_ped_yaw'] = ped_yaw

    # 速度制御
    u[0] = params['vmax'] * np.tanh(params['TSCF']['k'] * goal_dis)
    u[0] = np.clip(u[0], params['vmin'], params['vmax'])

    # 角速度制御
    f1 = params['TSCF']['f1']
    f2 = params['TSCF']['f2']
    sind = np.sin(thetaD)
    cosd = np.cos(thetaD)
    mu = -f1 * z - f2 * (-sind)
    denom = 1 - z * kr

    if -sind < 0:
        f2 = -f2
    if np.abs(cosd) < 1e-6:
        cosd = 1e-6 * np.sign(cosd)
    if np.abs(denom) < 1e-6:
        denom = 1e-6 * np.sign(denom)

    u[1] = u[0] * ((-kr * cosd / denom) + (mu / cosd))
    u[1] = np.clip(u[1], params['wmin'], params['wmax'])

    # --- ローバーの入力条件 ---
    # 後ろ側にいる場合は入力反転
    if thetaD > np.pi/2 or thetaD < -np.pi/2:
        u = -u

    # 歩行者が横に2m移動したら停止
    if "ped_y_start" not in params:
        params["ped_y_start"] = params['ped_pos'][0, 1]
    ped_dy = abs(params['ped_pos'][0, 1] - params["ped_y_start"])
    if ped_dy > 2.0:
        u = np.zeros(2)

    # 目標位置周辺で角速度の振動を抑える
    if goal_dis < 0.3:
        u[1] *= 0.1

    return u, params