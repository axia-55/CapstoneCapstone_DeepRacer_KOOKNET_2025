import math

# https://github.com/dgnzlz/Capstone_AWS_DeepRacer

REWARD_FOR_FASTEST_TIME = 150
STANDARD_TIME = 12  # seconds (time that is easily done by model)
FASTEST_TIME = 10  # seconds (best time of 1st place on the track)


class Reward:
    def __init__(self, verbose=False):
        self.first_racingpoint_index = 0  # None
        self.verbose = verbose

    def reward_function(self, params):
        # Import package (needed for heading)
        # import math

        ################## HELPER FUNCTIONS ###################

        def get_distance(coor1, coor2):
            return math.sqrt((coor1[0] - coor2[0]) * (coor1[0] - coor2[0]) + (coor1[1] - coor2[1]) * (coor1[1] - coor2[1]))

        def get_radians(coor1, coor2):
            return math.atan2((coor2[1] - coor1[1]), (coor2[0] - coor1[0]))

        def get_degrees(coor1, coor2):
            return math.degrees(get_radians(coor1, coor2))

        def get_diff_radians(angle1, angle2):
            diff = (angle1 - angle2) % (2.0 * math.pi)

            if diff >= math.pi:
                diff -= 2.0 * math.pi

            return diff

        def get_diff_degrees(angle1, angle2):
            return math.degrees(get_diff_radians(angle1, angle2))

        def up_sample(waypoints, factor=20):
            p = waypoints
            n = len(p)

            return [
                [
                    i / factor * p[int((j + 1) % n)][0] + (1 - i / factor) * p[j][0],
                    i / factor * p[int((j + 1) % n)][1] + (1 - i / factor) * p[j][1],
                ]
                for j in range(n)
                for i in range(factor)
            ]

        def get_distance_list(car, waypoints):
            dist_list = []
            min_dist = float("inf")
            min_idx = -1

            for i, waypoint in enumerate(waypoints):
                dist = get_distance(car, waypoint)
                if dist < min_dist:
                    min_dist = dist
                    min_idx = i
                dist_list.append(dist)

            return dist_list, min_dist, min_idx, len(waypoints)

        def detect_bot(params):
            car = [params["x"], params["y"]]

            heading = math.radians(params["heading"])
            track_width = params["track_width"]
            is_reversed = params["is_reversed"]

            objects_location = params["objects_location"]
            objects_left_of_center = params["objects_left_of_center"]

            warned = False
            is_inner = False

            bot_idx = -1
            bot_dist = float("inf")

            for i, location in enumerate(objects_location):
                dist = get_distance(car, location)

                angle = get_radians(car, location)

                diff = abs(get_diff_degrees(heading, angle))

                if dist < track_width and diff < 120:
                    warned = True

                    if dist < bot_dist:
                        bot_idx = i
                        bot_dist = dist

            if warned:
                if is_reversed:
                    if objects_left_of_center[bot_idx] == False:
                        is_inner = True
                else:
                    if objects_left_of_center[bot_idx]:
                        is_inner = True

            return warned, is_inner, bot_dist

        ################## HELPER FUNCTIONS ###################

        def dist_2_points(x1, x2, y1, y2):
            return abs(abs(x1 - x2) ** 2 + abs(y1 - y2) ** 2) ** 0.5

        def closest_2_racing_points_index(racing_coords, car_coords):
            # Calculate all distances to racing points
            distances = []
            for i in range(len(racing_coords)):
                distance = dist_2_points(
                    x1=racing_coords[i][0],
                    x2=car_coords[0],
                    y1=racing_coords[i][1],
                    y2=car_coords[1],
                )
                distances.append(distance)

            # Get index of the closest racing point
            closest_index = distances.index(min(distances))

            # Get index of the second closest racing point
            distances_no_closest = distances.copy()
            distances_no_closest[closest_index] = 999
            second_closest_index = distances_no_closest.index(min(distances_no_closest))

            return [closest_index, second_closest_index]

        def dist_to_racing_line(closest_coords, second_closest_coords, car_coords):
            # Calculate the distances between 2 closest racing points
            a = abs(
                dist_2_points(
                    x1=closest_coords[0],
                    x2=second_closest_coords[0],
                    y1=closest_coords[1],
                    y2=second_closest_coords[1],
                )
            )

            # Distances between car and closest and second closest racing point
            b = abs(
                dist_2_points(
                    x1=car_coords[0],
                    x2=closest_coords[0],
                    y1=car_coords[1],
                    y2=closest_coords[1],
                )
            )
            c = abs(
                dist_2_points(
                    x1=car_coords[0],
                    x2=second_closest_coords[0],
                    y1=car_coords[1],
                    y2=second_closest_coords[1],
                )
            )

            # Calculate distance between car and racing line (goes through 2 closest racing points)
            # try-except in case a=0 (rare bug in DeepRacer)
            try:
                distance = abs(-(a**4) + 2 * (a**2) * (b**2) + 2 * (a**2) * (c**2) - (b**4) + 2 * (b**2) * (c**2) - (c**4)) ** 0.5 / (2 * a)
            except:
                distance = b

            return distance

        # Calculate which one of the closest racing points is the next one and which one the previous one
        def next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading):
            # Virtually set the car more into the heading direction
            heading_vector = [
                math.cos(math.radians(heading)),
                math.sin(math.radians(heading)),
            ]
            new_car_coords = [
                car_coords[0] + heading_vector[0],
                car_coords[1] + heading_vector[1],
            ]

            # Calculate distance from new car coords to 2 closest racing points
            distance_closest_coords_new = dist_2_points(
                x1=new_car_coords[0],
                x2=closest_coords[0],
                y1=new_car_coords[1],
                y2=closest_coords[1],
            )
            distance_second_closest_coords_new = dist_2_points(
                x1=new_car_coords[0],
                x2=second_closest_coords[0],
                y1=new_car_coords[1],
                y2=second_closest_coords[1],
            )

            if distance_closest_coords_new <= distance_second_closest_coords_new:
                next_point_coords = closest_coords
                prev_point_coords = second_closest_coords
            else:
                next_point_coords = second_closest_coords
                prev_point_coords = closest_coords

            return [next_point_coords, prev_point_coords]

        def racing_direction_diff(closest_coords, second_closest_coords, car_coords, heading):
            # Calculate the direction of the center line based on the closest waypoints
            next_point, prev_point = next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading)

            # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
            track_direction = math.atan2(next_point[1] - prev_point[1], next_point[0] - prev_point[0])

            # Convert to degree
            track_direction = math.degrees(track_direction)

            # Calculate the difference between the track direction and the heading direction of the car
            direction_diff = abs(track_direction - heading)
            if direction_diff > 180:
                direction_diff = 360 - direction_diff

            return direction_diff

        # Gives back indexes that lie between start and end index of a cyclical list
        # (start index is included, end index is not)
        def indexes_cyclical(start, end, array_len):
            if end < start:
                end += array_len

            return [index % array_len for index in range(start, end)]

        # Calculate how long car would take for entire lap, if it continued like it did until now
        def projected_time(first_index, closest_index, step_count, times_list):
            # Calculate how much time has passed since start
            current_actual_time = (step_count - 1) / 15

            # Calculate which indexes were already passed
            indexes_traveled = indexes_cyclical(first_index, closest_index, len(times_list))

            # Calculate how much time should have passed if car would have followed optimals
            current_expected_time = sum([times_list[i] for i in indexes_traveled])

            # Calculate how long one entire lap takes if car follows optimals
            total_expected_time = sum(times_list)

            # Calculate how long car would take for entire lap, if it continued like it did until now
            try:
                projected_time = (current_actual_time / current_expected_time) * total_expected_time
            except:
                projected_time = 9999

            return projected_time

        #################### RACING LINE ######################

        # Optimal racing line for the Spain track
        # Each row: [x,y,speed,timeFromPreviousPoint]
        racing_track = []

        racing_track_ccw = [
            [8.54861, 3.17394, 4.0, 0.05041],
            [8.34696, 3.17288, 3.27921, 0.06149],
            [8.14534, 3.16945, 1.90054, 0.1061],
            [7.9437, 3.17026, 1.90054, 0.1061],
            [7.74208, 3.17388, 1.90054, 0.1061],
            [7.54044, 3.17271, 1.80747, 0.11156],
            [7.33883, 3.16878, 1.44104, 0.13993],
            [7.13746, 3.17531, 1.36483, 0.14761],
            [6.93942, 3.2128, 1.36483, 0.14769],
            [6.74666, 3.27162, 1.36483, 0.14766],
            [6.5565, 3.33852, 1.36483, 0.14771],
            [6.35818, 3.37203, 1.36483, 0.14736],
            [6.15836, 3.3517, 1.36483, 0.14716],
            [5.97304, 3.27302, 1.53252, 0.13137],
            [5.7831, 3.20663, 1.53252, 0.1313],
            [5.58254, 3.18686, 2.71501, 0.07423],
            [5.38186, 3.16761, 2.71501, 0.07426],
            [5.18026, 3.16361, 4.0, 0.05041],
            [4.97861, 3.16373, 4.0, 0.05041],
            [4.777, 3.16781, 4.0, 0.05041],
            [4.57536, 3.16932, 4.0, 0.05041],
            [4.37371, 3.16804, 4.0, 0.05041],
            [4.17206, 3.16802, 4.0, 0.05041],
            [3.97041, 3.16883, 4.0, 0.05041],
            [3.76876, 3.16972, 4.0, 0.05041],
            [3.56711, 3.17069, 4.0, 0.05041],
            [3.36546, 3.17165, 4.0, 0.05041],
            [3.16381, 3.17214, 4.0, 0.05041],
            [2.96216, 3.1739, 3.35493, 0.06011],
            [2.76053, 3.17645, 2.005, 0.10057],
            [2.55888, 3.17661, 1.87099, 0.10778],
            [2.35727, 3.17291, 1.87099, 0.10777],
            [2.15587, 3.16279, 1.77942, 0.11332],
            [1.95429, 3.15845, 1.77942, 0.11332],
            [1.75279, 3.16412, 1.27533, 0.15806],
            [1.55414, 3.19761, 1.27177, 0.1584],
            [1.36342, 3.2622, 1.27177, 0.15834],
            [1.17883, 3.34299, 1.27177, 0.15844],
            [0.98303, 3.39011, 1.27177, 0.15835],
            [0.78205, 3.40232, 1.27177, 0.15833],
            [0.58971, 3.34639, 1.27177, 0.1575],
            [0.42712, 3.2283, 1.30061, 0.1545],
            [0.30783, 3.06674, 1.30061, 0.15442],
            [0.24734, 2.87493, 2.28296, 0.0881],
            [0.20758, 2.67724, 1.67975, 0.12004],
            [0.16923, 2.47927, 1.41011, 0.143],
            [0.12879, 2.28172, 1.38895, 0.14518],
            [0.08933, 2.08396, 1.36187, 0.14807],
            [0.05045, 1.88609, 1.36187, 0.14807],
            [0.01244, 1.68806, 1.36187, 0.14807],
            [0.01407, 1.48727, 1.36187, 0.14744],
            [0.07138, 1.29448, 1.36187, 0.14768],
            [0.18121, 1.12628, 1.36187, 0.14751],
            [0.33609, 0.99815, 1.44968, 0.13865],
            [0.51934, 0.91513, 1.86198, 0.10805],
            [0.71359, 0.86251, 2.26415, 0.08889],
            [0.9113, 0.82284, 2.26415, 0.08906],
            [1.11132, 0.79873, 2.26415, 0.08898],
            [1.3122, 0.78331, 2.13143, 0.09452],
            [1.51033, 0.74612, 2.13143, 0.09458],
            [1.70378, 0.68944, 2.13143, 0.09458],
            [1.89507, 0.62562, 2.13143, 0.09461],
            [2.08593, 0.56055, 2.13143, 0.09461],
            [2.27928, 0.50595, 1.90715, 0.10534],
            [2.47854, 0.47534, 1.90715, 0.10571],
            [2.679, 0.45341, 1.90715, 0.10573],
            [2.87966, 0.43352, 1.72417, 0.11696],
            [3.08048, 0.41525, 1.5441, 0.13059],
            [3.28179, 0.41293, 1.5441, 0.13038],
            [3.48132, 0.44145, 1.5441, 0.13053],
            [3.67497, 0.49666, 1.5441, 0.13041],
            [3.86265, 0.57016, 1.5441, 0.13054],
            [4.06043, 0.60725, 1.5441, 0.13032],
            [4.26142, 0.59741, 1.34276, 0.14986],
            [4.45573, 0.54476, 1.34276, 0.14993],
            [4.63522, 0.45377, 1.34276, 0.14987],
            [4.8142, 0.36131, 1.34276, 0.15003],
            [5.00562, 0.29867, 1.34276, 0.15],
            [5.20439, 0.26524, 1.34276, 0.15011],
            [5.40246, 0.29377, 1.37559, 0.14548],
            [5.58469, 0.37907, 1.51497, 0.13281],
            [5.74973, 0.49457, 1.51497, 0.13297],
            [5.89772, 0.63132, 1.51497, 0.133],
            [6.02738, 0.78567, 1.51497, 0.13306],
            [6.18866, 0.90566, 1.51497, 0.13269],
            [6.37435, 0.98337, 1.74266, 0.11551],
            [6.57115, 1.02583, 1.53436, 0.13121],
            [6.77196, 1.04399, 1.51902, 0.13273],
            [6.97354, 1.04728, 1.29586, 0.15558],
            [7.17288, 1.01777, 1.29586, 0.1555],
            [7.37233, 0.9881, 1.27476, 0.15819],
            [7.56838, 0.9427, 1.25276, 0.16063],
            [7.74813, 0.85258, 1.25276, 0.1605],
            [7.90125, 0.72181, 1.25276, 0.16073],
            [8.00254, 0.54962, 1.25276, 0.15947],
            [8.0774, 0.36339, 1.25276, 0.16021],
            [8.21063, 0.21329, 1.25276, 0.1602],
            [8.38762, 0.11911, 1.39029, 0.14421],
            [8.58513, 0.07905, 3.48573, 0.05782],
            [8.78439, 0.0481, 1.40837, 0.14318],
            [8.98469, 0.02489, 1.40837, 0.14317],
            [9.18583, 0.01077, 1.26391, 0.15954],
            [9.38736, 0.00586, 1.22846, 0.1641],
            [9.38736, 0.00586, 1.22846, 0.0],
            [9.58741, 0.02668, 1.22846, 0.16372],
            [9.77374, 0.10227, 1.22846, 0.16368],
            [9.93298, 0.22484, 1.22846, 0.16358],
            [10.04023, 0.39388, 1.22846, 0.16296],
            [10.07921, 0.59097, 1.53497, 0.13089],
            [10.07081, 0.7922, 2.64898, 0.07603],
            [10.04642, 0.99236, 4.0, 0.05041],
            [10.02742, 1.19311, 3.7483, 0.0538],
            [10.00581, 1.3936, 3.7483, 0.0538],
            [9.98378, 1.59404, 2.79105, 0.07225],
            [9.96141, 1.79445, 1.32033, 0.15273],
            [9.94145, 1.99511, 1.2, 0.16804],
            [9.91647, 2.1951, 1.2, 0.16795],
            [9.88355, 2.39404, 1.2, 0.16804],
            [9.85291, 2.59335, 1.2, 0.16804],
            [9.80824, 2.78935, 1.2, 0.16752],
            [9.70432, 2.96096, 1.2, 0.16719],
            [9.54351, 3.0803, 1.34135, 0.14929],
            [9.35301, 3.14517, 1.71542, 0.11732],
            [9.15341, 3.17298, 2.10538, 0.09572],
            [8.9519, 3.17549, 4.0, 0.05038],
            [8.75026, 3.17344, 4.0, 0.05041],
        ]


        racing_track_cw = [
            [8.54861, 3.17394, 1.34135, 0.15033],
            [8.75026, 3.17344, 1.2, 0.16804],
            [8.9519, 3.17549, 1.2, 0.16804],
            [9.15341, 3.17298, 1.2, 0.16794],
            [9.35301, 3.14517, 1.2, 0.16793],
            [9.54351, 3.0803, 1.2, 0.16771],
            [9.70432, 2.96096, 1.2, 0.16687],
            [9.80824, 2.78935, 1.32033, 0.15195],
            [9.85291, 2.59335, 2.79105, 0.07202],
            [9.88355, 2.39404, 3.7483, 0.0538],
            [9.91647, 2.1951, 3.7483, 0.0538],
            [9.94145, 1.99511, 4.0, 0.05039],
            [9.96141, 1.79445, 2.64898, 0.07612],
            [9.98378, 1.59404, 1.53497, 0.13137],
            [10.00581, 1.3936, 1.22846, 0.16415],
            [10.02742, 1.19311, 1.22846, 0.16415],
            [10.04642, 0.99236, 1.22846, 0.16415],
            [10.07081, 0.7922, 1.22846, 0.16414],
            [10.07921, 0.59097, 1.22846, 0.16395],
            [10.04023, 0.39388, 1.22846, 0.16354],
            [9.93298, 0.22484, 1.26391, 0.15839],
            [9.77374, 0.10227, 1.40837, 0.14269],
            [9.58741, 0.02668, 1.40837, 0.14277],
            [9.38736, 0.00586, 3.48573, 0.0577],
            [9.38736, 0.00586, 1.39029, 0.0],
            [9.18583, 0.01077, 1.25276, 0.16092],
            [8.98469, 0.02489, 1.25276, 0.16096],
            [8.78439, 0.0481, 1.25276, 0.16095],
            [8.58513, 0.07905, 1.25276, 0.16096],
            [8.38762, 0.11911, 1.25276, 0.16087],
            [8.21063, 0.21329, 1.25276, 0.16004],
            [8.0774, 0.36339, 1.27476, 0.15744],
            [8.00254, 0.54962, 1.29586, 0.15489],
            [7.90125, 0.72181, 1.29586, 0.15416],
            [7.74813, 0.85258, 1.51902, 0.13256],
            [7.56838, 0.9427, 1.53436, 0.13105],
            [7.37233, 0.9881, 1.74266, 0.11548],
            [7.17288, 1.01777, 1.51497, 0.1331],
            [6.97354, 1.04728, 1.51497, 0.13301],
            [6.77196, 1.04399, 1.51497, 0.13308],
            [6.57115, 1.02583, 1.51497, 0.13309],
            [6.37435, 0.98337, 1.51497, 0.13289],
            [6.18866, 0.90566, 1.37559, 0.14633],
            [6.02738, 0.78567, 1.34276, 0.14971],
            [5.89772, 0.63132, 1.34276, 0.15013],
            [5.74973, 0.49457, 1.34276, 0.15006],
            [5.58469, 0.37907, 1.34276, 0.15002],
            [5.40246, 0.29377, 1.34276, 0.14984],
            [5.20439, 0.26524, 1.34276, 0.14903],
            [5.00562, 0.29867, 1.5441, 0.13054],
            [4.8142, 0.36131, 1.5441, 0.13044],
            [4.63522, 0.45377, 1.5441, 0.13046],
            [4.45573, 0.54476, 1.5441, 0.13033],
            [4.26142, 0.59741, 1.5441, 0.13038],
            [4.06043, 0.60725, 1.5441, 0.13032],
            [3.86265, 0.57016, 1.72417, 0.11671],
            [3.67497, 0.49666, 1.90715, 0.10569],
            [3.48132, 0.44145, 1.90715, 0.10559],
            [3.28179, 0.41293, 1.90715, 0.10568],
            [3.08048, 0.41525, 2.13143, 0.09445],
            [2.87966, 0.43352, 2.13143, 0.09461],
            [2.679, 0.45341, 2.13143, 0.09461],
            [2.47854, 0.47534, 2.13143, 0.09461],
            [2.27928, 0.50595, 2.13143, 0.09459],
            [2.08593, 0.56055, 2.26415, 0.08873],
            [1.89507, 0.62562, 2.26415, 0.08906],
            [1.70378, 0.68944, 2.26415, 0.08906],
            [1.51033, 0.74612, 1.86198, 0.10827],
            [1.3122, 0.78331, 1.44968, 0.13906],
            [1.11132, 0.79873, 1.36187, 0.14794],
            [0.9113, 0.82284, 1.36187, 0.14794],
            [0.71359, 0.86251, 1.36187, 0.14807],
            [0.51934, 0.91513, 1.36187, 0.14778],
            [0.33609, 0.99815, 1.36187, 0.14772],
            [0.18121, 1.12628, 1.36187, 0.14759],
            [0.07138, 1.29448, 1.38895, 0.14464],
            [0.01407, 1.48727, 1.41011, 0.14263],
            [0.01244, 1.68806, 1.67975, 0.11954],
            [0.05045, 1.88609, 2.28296, 0.08833],
            [0.08933, 2.08396, 1.30061, 0.15504],
            [0.12879, 2.28172, 1.30061, 0.15505],
            [0.16923, 2.47927, 1.27177, 0.15856],
            [0.20758, 2.67724, 1.27177, 0.15856],
            [0.24734, 2.87493, 1.27177, 0.15855],
            [0.30783, 3.06674, 1.27177, 0.15814],
            [0.42712, 3.2283, 1.27177, 0.15792],
            [0.58971, 3.34639, 1.27177, 0.158],
            [0.78205, 3.40232, 1.27533, 0.15706],
            [0.98303, 3.39011, 1.77942, 0.11316],
            [1.17883, 3.34299, 1.77942, 0.11318],
            [1.36342, 3.2622, 1.87099, 0.10769],
            [1.55414, 3.19761, 1.87099, 0.10763],
            [1.75279, 3.16412, 2.005, 0.10047],
            [1.95429, 3.15845, 3.35493, 0.06008],
            [2.15587, 3.16279, 4.0, 0.05041],
            [2.35727, 3.17291, 4.0, 0.05041],
            [2.55888, 3.17661, 4.0, 0.05041],
            [2.76053, 3.17645, 4.0, 0.05041],
            [2.96216, 3.1739, 4.0, 0.05041],
            [3.16381, 3.17214, 4.0, 0.05041],
            [3.36546, 3.17165, 4.0, 0.05041],
            [3.56711, 3.17069, 4.0, 0.05041],
            [3.76876, 3.16972, 4.0, 0.05041],
            [3.97041, 3.16883, 4.0, 0.05041],
            [4.17206, 3.16802, 4.0, 0.05041],
            [4.37371, 3.16804, 2.71501, 0.07427],
            [4.57536, 3.16932, 2.71501, 0.07427],
            [4.777, 3.16781, 1.53252, 0.13158],
            [4.97861, 3.16373, 1.53252, 0.13158],
            [5.18026, 3.16361, 1.36483, 0.14775],
            [5.38186, 3.16761, 1.36483, 0.14774],
            [5.58254, 3.18686, 1.36483, 0.14771],
            [5.7831, 3.20663, 1.36483, 0.14766],
            [5.97304, 3.27302, 1.36483, 0.14743],
            [6.15836, 3.3517, 1.36483, 0.14751],
            [6.35818, 3.37203, 1.44104, 0.13938],
            [6.5565, 3.33852, 1.80747, 0.11127],
            [6.74666, 3.27162, 1.90054, 0.10607],
            [6.93942, 3.2128, 1.90054, 0.10604],
            [7.13746, 3.17531, 1.90054, 0.10606],
            [7.33883, 3.16878, 3.27921, 0.06144],
            [7.54044, 3.17271, 4.0, 0.05041],
            [7.74208, 3.17388, 4.0, 0.05041],
            [7.9437, 3.17026, 4.0, 0.05041],
            [8.14534, 3.16945, 2.10538, 0.09578],
            [8.34696, 3.17288, 1.71542, 0.11755],
        ]

        ################## INPUT PARAMETERS ###################

        # Read all input parameters
        # all_wheels_on_track = params["all_wheels_on_track"]
        x = params["x"]
        y = params["y"]
        # distance_from_center = params["distance_from_center"]
        # is_left_of_center = params["is_left_of_center"]
        heading = params["heading"]
        progress = params["progress"]
        steps = params["steps"]
        speed = params["speed"]
        steering_angle = params["steering_angle"]
        track_width = params["track_width"]
        # waypoints = params["waypoints"]
        # closest_waypoints = params["closest_waypoints"]
        is_offtrack = params["is_offtrack"]
        is_reversed = params["is_reversed"]

        # closest_objects = params["closest_objects"]

        ############### OPTIMAL X,Y,SPEED,TIME ################

        # track = racing_track

        if is_reversed:
            track = racing_track_cw
        else:
            track = racing_track_ccw

        # if closest_objects:
        #     warned, is_inner, _ = detect_bot(params)

        #     if warned:
        #         if is_inner:
        #             track = outer_track
        #         else:
        #             track = inner_track

        # Get closest indexes for racing line (and distances to all points on racing line)
        closest_index, second_closest_index = closest_2_racing_points_index(track, [x, y])

        # Get optimal [x, y, speed, time] for closest and second closest index
        optimals = track[closest_index]
        optimals_second = track[second_closest_index]

        # Save first racingpoint of episode for later
        if self.verbose == True:
            self.first_racingpoint_index = 0  # this is just for testing purposes
        if steps == 1:
            self.first_racingpoint_index = closest_index

        ################ REWARD AND PUNISHMENT ################

        ## Define the default reward ##
        reward = 1
        MIN_REWARD = 1e-2

        ## Reward if car goes close to optimal racing line ##
        DISTANCE_MULTIPLE = 3
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(MIN_REWARD, 1 - (dist / (track_width * 0.5)))
        reward += distance_reward * DISTANCE_MULTIPLE

        ## Reward if speed is close to optimal speed ##
        SPEED_DIFF_NO_REWARD = 1
        SPEED_MULTIPLE = 3
        speed_diff = abs(optimals[2] - speed)
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
            # so, we do not punish small deviations from optimal speed
            speed_reward = (1 - (speed_diff / (SPEED_DIFF_NO_REWARD)) ** 2) ** 2
        else:
            speed_reward = 0
        reward += speed_reward * SPEED_MULTIPLE

        # Reward if less steps
        REWARD_PER_STEP_FOR_FASTEST_TIME = 1.5
        # STANDARD_TIME = 50  # seconds (time that is easily done by model)
        # FASTEST_TIME = 20  # seconds (best time of 1st place on the track)
        times_list = [row[3] for row in track]
        projected_time = projected_time(self.first_racingpoint_index, closest_index, steps, times_list)
        try:
            steps_prediction = projected_time * 15 + 1
            reward_prediction = max(
                MIN_REWARD,
                (-REWARD_PER_STEP_FOR_FASTEST_TIME * (FASTEST_TIME) / (STANDARD_TIME - FASTEST_TIME)) * (steps_prediction - (STANDARD_TIME * 15 + 1)),
            )
            steps_reward = min(REWARD_PER_STEP_FOR_FASTEST_TIME, reward_prediction / steps_prediction)
        except:
            steps_reward = 0
        reward += steps_reward

        # Zero reward if obviously wrong direction (e.g. spin)
        direction_diff = racing_direction_diff(optimals[0:2], optimals_second[0:2], [x, y], heading)
        if direction_diff > 30 or abs(steering_angle) > 20:
            reward = MIN_REWARD
        else:
            reward += 1.1 - (direction_diff / 30)

        # Zero reward of obviously too slow
        speed_diff_zero = optimals[2] - speed
        if speed_diff_zero > 0.5:
            reward = MIN_REWARD

        ## Incentive for finishing the lap in less steps ##
        # should be adapted to track length and other rewards
        # REWARD_FOR_FASTEST_TIME = 300
        # STANDARD_TIME = 50  # seconds (time that is easily done by model)
        # FASTEST_TIME = 20  # seconds (best time of 1st place on the track)
        if progress > 99.5:
            finish_reward = max(
                MIN_REWARD,
                (-REWARD_FOR_FASTEST_TIME / (15 * (STANDARD_TIME - FASTEST_TIME))) * (steps - STANDARD_TIME * 15),
            )
        else:
            finish_reward = 0
        reward += finish_reward

        ## Zero reward if off track ##
        if is_offtrack == True:
            reward = MIN_REWARD

        ####################### VERBOSE #######################
        if self.verbose == True:
            print("Closest index: %i" % closest_index)
            print("Distance to racing line: %f" % dist)
            print("=== Distance reward (w/out multiple): %f ===" % (distance_reward))
            print("Optimal speed: %f" % optimals[2])
            print("Speed difference: %f" % speed_diff)
            print("=== Speed reward (w/out multiple): %f ===" % speed_reward)
            print("Direction difference: %f" % direction_diff)
            print("Predicted time: %f" % projected_time)
            print("=== Steps reward: %f ===" % steps_reward)
            print("=== Finish reward: %f ===" % finish_reward)

        #################### RETURN REWARD ####################

        # Always return a float value
        return float(reward)


reward_object = Reward()  # add parameter verbose=True to get noisy output for testing


def reward_function(params):
    return reward_object.reward_function(params)
