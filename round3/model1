import math

def reward_function(p):
    # 즉시 실패 조건
    if p['is_offtrack'] or p['is_reversed']:
        return 1e-3

    # ---------- 1) 중앙선 거리 보상 ----------
    track_width   = p['track_width']
    dist_center   = p['distance_from_center']
    center_norm   = dist_center / (track_width * 0.5)
    center_reward = max(0.0, 1 - center_norm**2)

    # ---------- 2) 헤딩(진행 방향) 일치 보상 ----------
    waypoints            = p['waypoints']
    prev_i, next_i       = p['closest_waypoints']
    track_dx = waypoints[next_i][0] - waypoints[prev_i][0]
    track_dy = waypoints[next_i][1] - waypoints[prev_i][1]
    track_yaw = math.atan2(track_dy, track_dx)
    diff        = abs(track_yaw - math.radians(p['heading']))
    diff        = min(diff, 2*math.pi - diff)
    heading_reward = max(0.0, 1 - diff/(math.pi/2))

    # ---------- 3) 속도·조향 상호작용 ----------
    speed   = p['speed']          # m/s
    steer   = abs(p['steering_angle'])
    v_norm  = speed / 1.6         # <-- v_max를 1.6 m/s로 수정
    speed_penalty = 1 - (steer/30.0)
    speed_reward  = max(0.0, v_norm * speed_penalty)

    # ---------- 4) 진행도(완주 유도) ----------
    progress_reward = p['progress'] / 100.0

    # ---------- 5) 종합 ----------
    w_center, w_heading, w_speed, w_progress = 0.25, 0.25, 0.25, 0.25
    reward = (
        1e-3 +
        w_center   * center_reward   +
        w_heading  * heading_reward  +
        w_speed    * speed_reward    +
        w_progress * progress_reward
    )
    return float(reward)
