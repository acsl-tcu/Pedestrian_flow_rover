import numpy as np
import cvxpy as cp

def CBF(x, u, params):
    """
    QP版CBF（Wv/Ww 非対称重み）
    """
    
    # paramsの引き渡し
    ped_pos_all = params['ped_pos_all'] # 歩行者位置リスト
    r = params['CBFr'] # 進入禁止領域半径
    vmax = params['vmax'] # 上限速度
    wmax = params['wmax'] # 上限角速度

    # パラメータ設定
    l  = 0.3 # ロボット前方評価点
    Wv = 0.2 # 速度重み(値を増やすと元の入力を維持)
    Ww = 0.01 # 角速度重み(値を増やすと元の入力を維持)
    theta = x[2] # 姿勢角
    u_opt = cp.Variable(2) # 最適化変数[v, w]

    # 目的関数
    J = (Wv * cp.square(u_opt[0] - u[0]) + Ww * cp.square(u_opt[1] - u[1]))

    # 入力制限
    constraints = [u_opt[0] >= 0.0, u_opt[0] <= vmax, cp.abs(u_opt[1]) <= wmax]

    # CBF制約
    for px, py in ped_pos_all:
        if np.isnan(px) or np.isnan(py):
            continue

        dx = x[0] + l * np.cos(theta) - px # 前方評価点から歩行者位置までのx偏差
        dy = x[1] + l * np.sin(theta) - py # 前方評価点から歩行者位置までのy偏差

        ddx = u_opt[0] * np.cos(theta) - l * np.sin(theta) * u_opt[1] # dxの一階微分
        ddy = u_opt[0] * np.sin(theta) + l * np.cos(theta) * u_opt[1] # dyの一階微分

        h = dx**2 + dy**2 - r**2 # 制御バリア関数
        dh = 2 * dx * ddx + 2 * dy * ddy # 制御バリア関数の一階微分

        # CBF不等式制約
        constraints += [dh >= -params['alpha'](h)]

    # 最適化問題を解く
    prob = cp.Problem(cp.Minimize(J), constraints)
    prob.solve(solver=cp.OSQP, warm_start=True)

    if u_opt.value is None:
        return u, params

    return u_opt.value, params


# import numpy as np

# def hdh(v, params):
#     """
#     h+dhの値, 拡張関数を考慮したhの値, hの変化率を計算
#     """
#     h = params['xe']**2 + params['ye']**2 - params['r']**2
#     alphah = params['alpha'](h)
#     dh = (2 * params['xe'] * np.cos(params['th']) + 2 * params['ye'] * np.sin(params['th'])) * v
#     val = alphah + dh

#     return val, alphah, dh


# def CBF(x, u, params):
#     """
#     すべての歩行者に対してCBFを適用し，危険なら停止
#     """
#     N = params['ped_pos'].shape[0]

#     ped_pos_all = params['ped_pos_all']

#     for i in range(N):
#         px, py = ped_pos_all[i]
#         # 歩行者位置がnanならスキップ
#         if np.isnan(px) or np.isnan(py):
#             continue

#         xe = x[0] - px
#         ye = x[1] - py
#         th = x[2]
#         r = params['CBFr']

#         param_i = {
#             'xe': xe,
#             'ye': ye,
#             'th': th,
#             'r': r,
#             'alpha': params['alpha']
#         }

#         h = xe**2 + ye**2 - r**2

#         # 危険領域に入っていたら速度を0にして即停止
#         if h < 0:
#             u[0] = 0.0
#             break

#         # ここをコメントアウトするとCBF内で停止するが，基本的にはコメントインしておく
#         val, _, _ = hdh(u[0], param_i)
#         if val < 0:
#             u[0] = 0.0
#             break

#     # 速度制限
#     vmax = params.get('vmax', np.inf)
#     if u[0] > vmax:
#         u[0] = vmax

#     # 後退なし
#     if u[0] < 0.0:
#         u[0] = 0.0

#     # 後退あり
#     # if u[0] < -vmax:
#     #     u[0] = -vmax

#     return u, params
