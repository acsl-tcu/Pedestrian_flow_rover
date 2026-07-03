std_msgs/Header header

int16 num                # 検出人数
string[] name            # 名前（未使用なら空文字）
float64[] cls            # クラス（未使用なら0.0）
float64[] id             # YOLOによるID
float64[] deg_all        # 全員の中心角度
float64[] degp_all       # 全員の右肩角度
float64[] degm_all       # 全員の左肩角度
float64[] kp_all         # 全員のキーポイント（flattenして1人分34要素）

# 選択ID情報
float64[] selected_kp    # 選択IDのキーポイント（1人分）
float64 selected_deg      # 選択IDの中心角度
float64 selected_degp     # 選択IDの右肩角度
float64 selected_degm     # 選択IDの左肩角度
float64 selected_id       # 選択ID
