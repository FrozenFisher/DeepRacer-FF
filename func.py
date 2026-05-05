def reward_function(params):
    # 线路分段：用 set 做 O(1) 成员判断
    left = set(range(22, 42)) | set(range(79, 88))
    center = set(range(1, 20)) | set(range(90, 103))
    right = set(range(48, 63))

    p = params["closest_waypoints"][1]
    track_w = params["track_width"]
    d_center = params["distance_from_center"]
    speed = params["speed"]
    steps = max(int(params["steps"]), 1)
    progress = params["progress"]

    reward = 1e-3

    # 保持在赛道上（主要约束）
    if not params["all_wheels_on_track"]:
        return 1e-3

    reward += 4.0

    # 单位步数推进进度：鼓励少步数跑完全程
    reward += (progress / steps) * 0.15

    # 直道（center 段）适度奖励速度，平方放大高速度但系数收敛，避免数值爆掉
    if p in center:
        reward += min(speed ** 2 * 2.0, 20.0)

    # 相对中心线的「贴线」程度，0~1
    half = max(track_w * 0.5, 1e-6)
    on_line = max(0.0, 1.0 - (d_center / half))

    # 行车线：按段落要求左右 / 中线
    if p in left and params["is_left_of_center"]:
        reward += 3.0 + 5.0 * on_line
    elif p in right and not params["is_left_of_center"]:
        reward += 3.0 + 5.0 * on_line
    elif p in center:
        # 直道鼓励贴近中心；略宽阈值内满分，之外按距离衰减
        if d_center < track_w * 0.12:
            reward += 6.0 + 2.0 * on_line
        else:
            reward += 1.5 * on_line
    else:
        reward -= 2.5

    # 弯道上略惩罚大方向盘，减少来回画龙
    if p not in center:
        reward -= 0.08 * abs(params["steering_angle"])

    return float(max(reward, 1e-3))
