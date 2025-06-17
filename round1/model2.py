import math

SIGHT = 0.9
MAX_REWARD = 3.0
MIN_REWARD = 1e-4          # off‑track 시에도 0 보상은 피하는 것이 안전

# ────────────────── 공용 헬퍼 함수 ──────────────────
def dist(p1, p2):
    return ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2) ** 0.5

def rect(r, theta):
    x = r * math.cos(math.radians(theta))
    y = r * math.sin(math.radians(theta))
    return x, y

def polar(x, y):
    r = (x**2 + y**2) ** 0.5
    theta = math.degrees(math.atan2(y, x))
    return r, theta

def angle_mod_360(angle):
    n = math.floor(angle / 360.0)
    a = angle - n * 360.0
    return a if a <= 180.0 else a - 360.0

def get_waypoints_ordered_in_driving_direction(params):
    wp = params["waypoints"]
    return list(reversed(wp)) if params["is_reversed"] else wp

def up_sample(wp, k=10):
    n = len(wp)
    return [[i/k*wp[(j+1)%n][0] + (1-i/k)*wp[j][0],
             i/k*wp[(j+1)%n][1] + (1-i/k)*wp[j][1]]
            for j in range(n) for i in range(k)]

def get_target_point(params):
    wp = up_sample(get_waypoints_ordered_in_driving_direction(params), 20)
    car  = [params["x"], params["y"]]
    dists = [dist(p, car) for p in wp]
    i_closest = dists.index(min(dists))
    n = len(wp)
    wp_rot = [wp[(i+i_closest)%n] for i in range(n)]
    r = params["track_width"] * SIGHT
    inside = [dist(p, car) < r for p in wp_rot]
    i_first_outside = inside.index(False)
    return wp_rot[i_first_outside]

def get_target_steering_degree(params):
    tx, ty = get_target_point(params)
    dx, dy = tx - params["x"], ty - params["y"]
    _, tgt_angle = polar(dx, dy)
    return angle_mod_360(tgt_angle - params["heading"])

# ────────────────── 개별 점수 ──────────────────
def score_speed(params):
    return max(min(params["speed"] / 2.5, MAX_REWARD), MIN_REWARD)

def score_steer_to_point_ahead(params):
    best = get_target_steering_degree(params)
    err  = (params["steering_angle"] - best) / 60.0
    return max(min(1.0 - abs(err), MAX_REWARD), MIN_REWARD)

def score_off_track(params):
    """트랙 이탈 여부 보상(패널티)"""
    on_track = params.get("all_wheels_on_track", not params.get("is_offtrack", False))
    return 1.0 if on_track else MIN_REWARD    # 이탈 시 강하게 깎기

# ────────────────── 최종 보상 ──────────────────
def reward_function(params):
    # 트랙을 벗어나면 즉시 최소 보상
    if params.get("is_offtrack", False) or not params.get("all_wheels_on_track", True):
        return float(MIN_REWARD)

    reward = 0.0
    reward += score_off_track(params)          # on/off‑track 확인 (on이면 1.0)
    reward += score_steer_to_point_ahead(params)
    reward += score_speed(params)
    return float(reward)
