def reward_function(params):
    all_wheels_on_track = params.get('all_wheels_on_track', True)
    distance_from_center = params.get('distance_from_center', 0.0)
    track_width = params.get('track_width', 1.0)
    is_crashed = params.get('is_crashed', False)

    reward = 1.0

    marker_1 = 0.1 * track_width
    marker_2 = 0.25 * track_width
    marker_3 = 0.5 * track_width

    if distance_from_center <= marker_1:
        reward = 1.0
    elif distance_from_center <= marker_2:
        reward = 0.7
    elif distance_from_center <= marker_3:
        reward = 0.4
    else:
        reward = 1e-3

    if not all_wheels_on_track:
        reward *= 0.1

    if is_crashed:
        reward = 1e-5

    return float(reward)
