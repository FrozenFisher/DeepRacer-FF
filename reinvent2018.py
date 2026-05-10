"""
re:Invent 2018 时期 DeepRacer 联赛常见约束：只能改奖励函数，动作空间与网络由平台固定。

本文件设计要点：
- 单文件、无本地 import，便于直接粘贴到控制台 / 上传训练。
- 不依赖 waypoint 编号（避免某些赛道专用 if/列表）。
- 中心线弧长前瞻 + 射线夹角选前瞻点（与 func.py 同源几何），直道偏远距离目标、急弯拉近，兼顾长直道速度与弯心稳定。
- 略强化「单位步数推进进度」，在只能调奖励时更有利于刷完全程与圈速。
"""

import math

VELOCITY = [1.2, 1.6, 2.3, 2.8, 4, 2.8, 2.3, 1.6, 1.2]
STEERS = [-30, -20, -10, -5, 0, 5, 10, 20, 30]
STtV = [(sp, st) for sp in VELOCITY for st in STEERS]


def _target_speed_for_steering(steer_deg, sttv):
    if not sttv:
        return 4.0
    dmin = min(abs(a[1] - steer_deg) for a in sttv)
    return max(a[0] for a in sttv if abs(a[1] - steer_deg) <= dmin + 1e-9)


def _dist(ax, ay, bx, by):
    return math.hypot(bx - ax, by - ay)


def _project_on_segment(ax, ay, bx, by, cx, cy):
    abx, aby = bx - ax, by - ay
    acx, acy = cx - ax, cy - ay
    ab2 = abx * abx + aby * aby
    if ab2 < 1e-18:
        return 0.0, ax, ay
    t = max(0.0, min(1.0, (acx * abx + acy * aby) / ab2))
    px = ax + t * abx
    py = ay + t * aby
    return t, px, py


def _angle_norm(diff):
    d = diff
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return abs(d)


def _angle_between_vecs(ax, ay, bx, by):
    la = math.hypot(ax, ay)
    lb = math.hypot(bx, by)
    if la < 1e-12 or lb < 1e-12:
        return 0.0
    c = max(-1.0, min(1.0, (ax * bx + ay * by) / (la * lb)))
    return math.acos(c)


def _forward_center_points(waypoints, prev_i, next_i, cx, cy, distances):
    n = len(waypoints)
    if n < 2:
        cx, cy = float(cx), float(cy)
        return [(cx, cy) for _ in distances]

    ax, ay = waypoints[prev_i]
    bx, by = waypoints[next_i]
    _, px, py = _project_on_segment(ax, ay, bx, by, cx, cy)

    order = sorted(range(len(distances)), key=lambda i: distances[i])
    targets = sorted(distances)
    out = [None] * len(distances)

    def place_on_segment(sx, sy, ex, ey, dist_from_start, need_d):
        sl = _dist(sx, sy, ex, ey)
        if sl < 1e-12:
            return ex, ey, dist_from_start
        u = (need_d - dist_from_start) / sl
        u = max(0.0, min(1.0, u))
        qx = sx + u * (ex - sx)
        qy = sy + u * (ey - sy)
        return qx, qy, dist_from_start + u * sl

    d_acc = 0.0
    ti = 0
    safety = n * 4 + 80
    sx, sy = px, py
    ex, ey = float(waypoints[next_i][0]), float(waypoints[next_i][1])
    seg_start_idx = next_i
    first_seg = True

    while ti < len(targets) and safety > 0:
        safety -= 1
        seg_len = _dist(sx, sy, ex, ey)
        while ti < len(targets) and d_acc + seg_len + 1e-9 >= targets[ti]:
            need = targets[ti]
            qx, qy, _ = place_on_segment(sx, sy, ex, ey, d_acc, need)
            out[order[ti]] = (qx, qy)
            ti += 1
        d_acc += seg_len
        if ti >= len(targets):
            break
        if first_seg:
            first_seg = False
            seg_start_idx = next_i
        else:
            seg_start_idx = (seg_start_idx + 1) % n
        en = (seg_start_idx + 1) % n
        sx, sy = float(waypoints[seg_start_idx][0]), float(waypoints[seg_start_idx][1])
        ex, ey = float(waypoints[en][0]), float(waypoints[en][1])

    last = (sx, sy)
    for i in range(len(out)):
        if out[i] is None:
            out[i] = last
        else:
            last = out[i]
    return out


def _pick_target_by_ray_angles(p_car, p1, p2, p3):
    cx, cy = p_car
    v1x, v1y = p1[0] - cx, p1[1] - cy
    v2x, v2y = p2[0] - cx, p2[1] - cy
    v3x, v3y = p3[0] - cx, p3[1] - cy
    a12 = _angle_between_vecs(v1x, v1y, v2x, v2y)
    a23 = _angle_between_vecs(v2x, v2y, v3x, v3y)
    spread = a12 + a23
    if spread < 4.2 / 360 * 2 * math.pi:
        return p3, spread
    if spread < 13.5 / 360 * 2 * math.pi:
        return p2, spread
    return p1, spread


def reward_function(params):
    p = params["closest_waypoints"][1]
    prev_p = params["closest_waypoints"][0]
    track_w = params["track_width"]
    d_center = params["distance_from_center"]
    speed = float(params["speed"])
    steering_angle = float(params["steering_angle"])
    steps = max(int(params["steps"]), 1)
    progress = params["progress"]

    cx = float(params["x"])
    cy = float(params["y"])
    heading = float(params["heading"])
    waypoints = params["waypoints"]

    reward = 1e-3

    if not params["all_wheels_on_track"]:
        return 1e-3

    reward += 4.0

    # 仅能通过奖励施压时：略强于 func.py，鼓励「更少步数走完同样进度」→ 有利于整体圈速
    reward += (progress / steps) * 4.0

    half = max(track_w * 0.5, 1e-6)
    on_line = max(0.0, 1.0 - (d_center / half))

    base_look = max(0.62 * track_w, 1e-6)
    d1, d2, d3 = base_look, 1.5 * base_look, 1.75 * base_look
    pts = _forward_center_points(waypoints, prev_p, p, cx, cy, [d1, d2, d3])
    target, spread = _pick_target_by_ray_angles((cx, cy), pts[0], pts[1], pts[2])
    bear = math.atan2(target[1] - cy, target[0] - cx)
    h_align = max(0.0, math.cos(_angle_norm(bear - math.radians(heading))))
    reward += 7.5 * h_align
    reward += 2.0 * max(0.0, 1.0 - spread / math.pi)

    reward += 1.2 * (on_line ** 2)

    target_speed = _target_speed_for_steering(steering_angle, STtV)
    den = max(target_speed, 0.25)
    speed_ratio_err = abs(speed - target_speed) / den
    speed_match = max(0.0, 1.0 - min(1.0, speed_ratio_err))

    steer_turn = min(1.0, abs(steering_angle) / 30.0)
    straight_factor = 0.2 + 0.8 * (1.0 - steer_turn ** 1.2)
    curve_factor = 0.25 + 0.75 * max(0.0, 1.0 - spread / (0.52 * math.pi))
    speed_scale = straight_factor * curve_factor

    reward += (speed ** 1.65) * 10.5 * speed_match * (0.35 + 0.65 * on_line) * speed_scale

    return float(max(reward, 1e-3))
