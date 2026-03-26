# GetTouchAct1.msg
# 触觉数据：每根手指的法向力与切向力（与 touch_data_ts 解析方式一致）

int32[] finger_ids
string[] finger_names
float64[] normal_forces     # 法向力（每指一个）
float64[] tangential_forces # 切向力（每指一个）
