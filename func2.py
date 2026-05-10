"""
简化奖励函数（func2）：面向样本效率，用少量平滑项让策略更快学到「往前走 + 贴中 + 对准路」。
适用于把训练长度设在约 1000 episode 左右的预算（仍需配合合理学习率、探索与轨迹长度等超参）。
"""

import math


def _heading_rad(params):
    """DeepRacer: heading 为度，0° 朝东，逆时针为正；与 atan2(dy, dx) 一致。"""
    return math.radians(float(params["heading"]))


def _bearing(cx, cy, tx, ty):
    return math.atan2(ty - cy, tx - cx)


def _angle_diff(a, b):
    d = a - b
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return abs(d)


def reward_function(params):
    if not params.get("all_wheels_on_track", False):
        return 1e-3

    progress = float(params["progress"])
    steps = max(int(params["steps"]), 1)
    track_w = max(float(params["track_width"]), 1e-6)
    d_center = float(params["distance_from_center"])
    speed = float(params["speed"])

    half = track_w * 0.5
    lateral = min(1.0, d_center / half)

    waypoints = params["waypoints"]
    n = len(waypoints)
    if n < 2:
        return float(max(1e-3, 1.0 + 8.0 * (progress / steps)))

    closest = params["closest_waypoints"]
    next_i = int(closest[1])
    cx = float(params["x"])
    cy = float(params["y"])
    h = _heading_rad(params)

    # 前瞻：当前段终点与下一段中点，稳定且比多点射线更简单
    nx, ny = float(waypoints[next_i][0]), float(waypoints[next_i][1])
    nn = (next_i + 1) % n
    tx = 0.5 * (nx + float(waypoints[nn][0]))
    ty = 0.5 * (ny + float(waypoints[nn][1]))
    bear = _bearing(cx, cy, tx, ty)
    align = max(0.0, math.cos(_angle_diff(bear, h)))

    reward = 1e-3
    reward += 2.0
    # 每步向前推进占一圈的比例 —— 强稠密信号，利于早期收敛
    reward += 12.0 * (progress / steps)
    # 贴中：二次型，梯度平滑
    reward += 4.0 * ((1.0 - lateral) ** 2)
    # 朝向与前瞻点一致
    reward += 3.5 * align
    # 仅在「大致对准且不太偏」时鼓励速度，避免先学成蠕动
    gate = align * (1.0 - lateral)
    reward += 2.0 * gate * min(speed / 4.0, 1.0)

    return float(max(reward, 1e-3))
