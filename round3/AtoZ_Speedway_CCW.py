import math

# https://github.com/dgnzlz/Capstone_AWS_DeepRacer

REWARD_FOR_FASTEST_TIME = 150
STANDARD_TIME = 12  # seconds (time that is easily done by model)
FASTEST_TIME = 8  # seconds (best time of 1st place on the track)


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
            [-1.58623, -1.47875, 2.6169, 0.08111],
            [-1.38236, -1.55008, 2.87489, 0.07513],
            [-1.16719, -1.60638, 3.18929, 0.06973],
            [-0.93995, -1.64964, 3.55567, 0.06506],
            [-0.69925, -1.68138, 4.0, 0.0607],
            [-0.44378, -1.70318, 4.0, 0.0641],
            [-0.17268, -1.71659, 3.28402, 0.08265],
            [0.11383, -1.72333, 2.66489, 0.10754],
            [0.41392, -1.72544, 2.27757, 0.13176],
            [0.72166, -1.72567, 1.99725, 0.15408],
            [1.0294, -1.72644, 1.78632, 0.17228],
            [1.32795, -1.71458, 1.60452, 0.18621],
            [1.60868, -1.68026, 1.46658, 0.19285],
            [1.86346, -1.61835, 1.32281, 0.19821],
            [2.08582, -1.52809, 1.2, 0.19999],
            [2.27132, -1.41161, 1.2, 0.18253],
            [2.41709, -1.27274, 1.2, 0.16778],
            [2.51988, -1.11565, 1.2, 0.15644],
            [2.57619, -0.94539, 1.2, 0.14944],
            [2.57768, -0.76801, 1.2, 0.14782],
            [2.50975, -0.59609, 1.5201, 0.12161],
            [2.39455, -0.43725, 1.726, 0.11368],
            [2.23762, -0.29541, 2.03996, 0.1037],
            [2.04603, -0.17147, 2.5154, 0.09071],
            [1.82829, -0.06333, 3.55947, 0.0683],
            [1.59614, 0.03523, 4.0, 0.06305],
            [1.33571, 0.15201, 4.0, 0.07135],
            [1.0774, 0.27335, 4.0, 0.07135],
            [0.82084, 0.39847, 4.0, 0.07136],
            [0.5657, 0.52665, 3.91788, 0.07288],
            [0.31172, 0.65729, 3.25378, 0.08778],
            [0.05863, 0.78986, 2.82574, 0.10111],
            [-0.19378, 0.92387, 2.52006, 0.1134],
            [-0.43568, 1.05018, 2.27813, 0.11979],
            [-0.6782, 1.16782, 2.04996, 0.13149],
            [-0.92148, 1.27033, 1.86083, 0.14187],
            [-1.16504, 1.35213, 1.69661, 0.15144],
            [-1.40767, 1.40835, 1.55259, 0.16041],
            [-1.64725, 1.43489, 1.55259, 0.15526],
            [-1.88069, 1.42819, 1.55259, 0.15042],
            [-2.10334, 1.38411, 1.55259, 0.14619],
            [-2.3087, 1.299, 1.55259, 0.14318],
            [-2.48752, 1.16933, 1.55259, 0.14227],
            [-2.62551, 0.99214, 1.87824, 0.11957],
            [-2.72916, 0.78557, 1.99312, 0.11596],
            [-2.79662, 0.55483, 2.0877, 0.11515],
            [-2.82488, 0.30634, 2.17509, 0.11498],
            [-2.81222, 0.05069, 2.25308, 0.11361],
            [-2.76089, -0.19917, 2.33804, 0.1091],
            [-2.67728, -0.43346, 2.36971, 0.10498],
            [-2.56753, -0.64682, 2.42593, 0.0989],
            [-2.43763, -0.83799, 2.44694, 0.09446],
            [-2.29161, -1.00711, 2.44886, 0.09124],
            [-2.13223, -1.15501, 2.44886, 0.08879],
            [-1.96118, -1.28252, 2.44886, 0.08712],
            [-1.77902, -1.38996, 2.52176, 0.08386],
        ]

        racing_track_cw = [
            [-3.58984, -0.37244, 1.78911, 0.09746],
            [-3.58212, -0.20115, 1.70007, 0.10086],
            [-3.55204, -0.03341, 1.6, 0.10651],
            [-3.49817, 0.12931, 1.6, 0.10713],
            [-3.41785, 0.28482, 1.6, 0.1094],
            [-3.30644, 0.42948, 1.6, 0.11412],
            [-3.15649, 0.55572, 2.08337, 0.09409],
            [-2.98187, 0.66645, 2.27481, 0.09089],
            [-2.78511, 0.76055, 2.47745, 0.08804],
            [-2.56862, 0.8372, 2.74203, 0.08376],
            [-2.33532, 0.89667, 3.07748, 0.07823],
            [-2.08826, 0.94019, 3.39245, 0.07395],
            [-1.82714, 0.96894, 3.44693, 0.07621],
            [-1.57128, 1.01329, 3.35658, 0.07736],
            [-1.32074, 1.07273, 3.24382, 0.07938],
            [-1.07564, 1.14695, 3.24382, 0.07895],
            [-0.83622, 1.23611, 3.24382, 0.07876],
            [-0.60299, 1.34088, 3.24382, 0.07882],
            [-0.3767, 1.46242, 3.87717, 0.06625],
            [-0.15548, 1.59578, 4.0, 0.06458],
            [0.06146, 1.73889, 3.7858, 0.06865],
            [0.2748, 1.89, 3.51148, 0.07445],
            [0.50676, 2.04072, 3.2773, 0.08441],
            [0.74241, 2.17843, 3.06637, 0.08901],
            [0.98152, 2.30112, 2.88117, 0.09328],
            [1.22366, 2.40678, 2.71775, 0.09721],
            [1.46817, 2.49348, 2.56012, 0.10134],
            [1.71412, 2.55931, 2.4076, 0.10575],
            [1.96024, 2.60242, 2.22846, 0.11212],
            [2.20483, 2.6211, 2.06527, 0.11878],
            [2.44574, 2.61353, 1.9188, 0.12561],
            [2.68019, 2.57786, 1.75875, 0.13484],
            [2.9043, 2.51133, 1.6225, 0.14408],
            [3.11283, 2.41145, 1.6225, 0.14251],
            [3.29893, 2.27617, 1.6225, 0.14181],
            [3.45246, 2.1031, 1.6225, 0.14259],
            [3.55899, 1.892, 2.10241, 0.11247],
            [3.63291, 1.66146, 2.26074, 0.10709],
            [3.67532, 1.41554, 2.44829, 0.10193],
            [3.68769, 1.15784, 2.65104, 0.09732],
            [3.67162, 0.89155, 2.86612, 0.09308],
            [3.62883, 0.61949, 3.01866, 0.09123],
            [3.56002, 0.34423, 3.15402, 0.08996],
            [3.46607, 0.06877, 3.27087, 0.08898],
            [3.34842, -0.20317, 3.3963, 0.08724],
            [3.20975, -0.46756, 3.46898, 0.08606],
            [3.05271, -0.72089, 3.51522, 0.08479],
            [2.87987, -0.96068, 3.52295, 0.0839],
            [2.69353, -1.18535, 3.505, 0.08328],
            [2.49564, -1.394, 3.42512, 0.08396],
            [2.28773, -1.58598, 3.34507, 0.0846],
            [2.07115, -1.7611, 3.18609, 0.08742],
            [1.84705, -1.91932, 3.03241, 0.09046],
            [1.61616, -2.06006, 2.82972, 0.09556],
            [1.37915, -2.1828, 2.63899, 0.10114],
            [1.13642, -2.28601, 2.63899, 0.09995],
            [0.8884, -2.368, 2.63899, 0.09899],
            [0.63544, -2.42589, 2.63899, 0.09833],
            [0.37814, -2.45614, 2.70931, 0.09562],
            [0.11792, -2.4599, 3.22355, 0.08073],
            [-0.14374, -2.44459, 3.51236, 0.07463],
            [-0.40626, -2.41286, 3.77866, 0.06998],
            [-0.66925, -2.36657, 4.0, 0.06676],
            [-0.93242, -2.3076, 4.0, 0.06742],
            [-1.19557, -2.23757, 4.0, 0.06808],
            [-1.4586, -2.15774, 4.0, 0.06872],
            [-1.72122, -2.06935, 3.82889, 0.07237],
            [-1.97892, -1.97498, 3.28968, 0.08342],
            [-2.22762, -1.87709, 2.90882, 0.09188],
            [-2.46668, -1.77276, 1.9588, 0.13316],
            [-2.69448, -1.6592, 1.9588, 0.12994],
            [-2.90817, -1.53408, 1.9588, 0.12642],
            [-3.10443, -1.39596, 1.9588, 0.12252],
            [-3.2617, -1.23774, 1.99287, 0.11194],
            [-3.38318, -1.06939, 1.98904, 0.10437],
            [-3.47374, -0.89624, 1.96309, 0.09954],
            [-3.53685, -0.72121, 1.93531, 0.09614],
            [-3.57502, -0.54617, 1.86669, 0.09598],
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
        if direction_diff > 30:
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
        if progress > 99.8:
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
