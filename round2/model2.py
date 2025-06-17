import math

def reward_function(params):
    distance_from_center = params.get('distance_from_center', 0.0)
    track_width = params.get('track_width', 1.0)
    is_crashed = params.get('is_crashed', False)
    all_wheels_on_track = params.get('all_wheels_on_track', True)
    lidar = params.get('lidar', [])

    reward = 1.0
    reward_avoid = 1.0

    normalized = 1 - (distance_from_center / (track_width / 2)) ** 2
    reward *= max(0.1, normalized)

    if len(lidar) == 108:
        center_dist = min(lidar[45:63])
        side_dist = min(min(lidar[0:15]), min(lidar[93:108]))

        if center_dist < 0.5:
            reward_avoid *= 0.4
        if center_dist < 0.3:
            reward_avoid *= 0.1
        if center_dist < 0.2:
            reward_avoid = 1e-3
        if side_dist < 0.2:
            reward_avoid *= 0.5

    reward *= reward_avoid

    if not all_wheels_on_track:
        reward *= 0.1

    if is_crashed:
        reward = 1e-5

    return float(reward)
