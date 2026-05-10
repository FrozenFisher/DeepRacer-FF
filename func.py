import math

VELOCITY = [1.2, 1.6, 2.3, 2.8, 4, 2.8, 2.3, 1.6, 1.2]
STEERS = [-30, -20, -10, -5, 0, 5, 10, 20, 30]
STtV = [(sp, st) for sp in VELOCITY for st in STEERS]


def _target_speed_for_steering(steer_deg, sttv):
    # 当前转向角下，在 STtV 中与该角最接近的转向列上，取该列允许的最大速度作为「目标速度」（同转角优先给足速）。
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
    """沿封闭赛道中心线，从车在 prev_i—next_i 段上的投影点向前量弧长，取各距离处的中心线点。"""
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
    # 第一段：投影点 -> waypoints[next_i]；之后逐段 waypoints[k] -> waypoints[k+1]
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
    """
    车 -> 三个中心线前瞻点 构成三条射线，用相邻夹角之和衡量弯道「张开」程度：
    夹角小（近似直行）选最远点；夹角大（急弯）选最近点；中间取 1.5 倍前瞻。
    """
    cx, cy = p_car
    v1x, v1y = p1[0] - cx, p1[1] - cy
    v2x, v2y = p2[0] - cx, p2[1] - cy
    v3x, v3y = p3[0] - cx, p3[1] - cy
    a12 = _angle_between_vecs(v1x, v1y, v2x, v2y)
    a23 = _angle_between_vecs(v2x, v2y, v3x, v3y)
    spread = a12 + a23
    # 夹角阈值（弧度）：缩小「最远点」区间，大弯/缓弯更多用中距前瞻，减少切弯过猛出线。
    # 原 6.88° / 16.53°；略压低最远档、拉开中档，急弯仍用最近点。
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

    reward += (progress / steps) * 1.5

    half = max(track_w * 0.5, 1e-6)
    on_line = max(0.0, 1.0 - (d_center / half))

    target_speed = _target_speed_for_steering(steering_angle, STtV)
    den = max(target_speed, 0.25)
    speed_ratio_err = abs(speed - target_speed) / den
    speed_match = max(0.0, 1.0 - min(1.0, speed_ratio_err))
    reward += (speed ** 2) * 13.0 * speed_match * (0.35 + 0.65 * on_line)

    # —— 1×、1.5×、1.75× 于 (0.62 * track_width) 的前瞻距，三条射线夹角决定目标点 ——
    base_look = max(0.62 * track_w, 1e-6)
    d1, d2, d3 = base_look, 1.5 * base_look, 1.75 * base_look
    pts = _forward_center_points(waypoints, prev_p, p, cx, cy, [d1, d2, d3])
    target, spread = _pick_target_by_ray_angles((cx, cy), pts[0], pts[1], pts[2])
    bear = math.atan2(target[1] - cy, target[0] - cx)
    h_align = max(0.0, math.cos(_angle_norm(bear - heading)))
    reward += 7.5 * h_align
    reward += 2.0 * max(0.0, 1.0 - spread / math.pi)

    return float(max(reward, 1e-3))
