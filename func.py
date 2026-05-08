def reward_function(params):
    # 线路分段：用 set 做 O(1) 成员判断
    left = set(range(22, 42)) | set(range(80, 88)) | set(range(104, 111)) | set(range(66, 71))
    center = set(range(1, 20)) | set(range(92, 103)) | set(range(114, 117)) | set(range(61, 62)) | set(range(74, 77))
    right = set(range(48, 58)) 

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
    reward += (progress / steps) * 100.0

    # 相对中心线的「贴线」程度，0~1（弯道速度项也要用，先算）
    half = max(track_w * 0.5, 1e-6)
    on_line = max(0.0, 1.0 - (d_center / half))

    # 直道：强速度奖励
    if p in center:
        reward += speed ** 2 * 14.0

    # 弯道：原先没有速度项，车会压到 ~1m/s 换稳定；在贴线好时给足速度分
    if p in left or p in right:
        line_factor = 0.35 + 0.65 * on_line
        reward += speed ** 2 * 9.0 * line_factor

    # 行车线：按段落要求左右 / 中线
    if p in left and params["is_left_of_center"]:
        reward += 5.0 + 5.0 * on_line
    elif p in right and not params["is_left_of_center"]:
        reward += 5.0 + 5.0 * on_line
    elif p in center:
        # 直道鼓励贴近中心；略宽阈值内满分，之外按距离衰减
        if d_center < track_w * 0.12:
            reward += 8.0 + 2.0 * on_line
        else:
            reward += 1.5 * on_line
    else:
        reward -= 6

    return float(max(reward, 1e-3))
