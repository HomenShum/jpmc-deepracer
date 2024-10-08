import math


class Reward:
    def __init__(self, verbose=True):
        self.first_racingpoint_index = None
        self.verbose = verbose

    def reward_function(self, params):

        # # Import package (needed for heading)
        # import math

        ################## HELPER FUNCTIONS ###################

        def dist_2_points(x1, x2, y1, y2):
            return abs(abs(x1-x2)**2 + abs(y1-y2)**2)**0.5

        def dist_2_points_hypot(x1, x2, y1, y2):
            return math.hypot(x1 - x2, y1 - y2)

        def calculate_heading(point1, point2):
            heading = math.degrees(math.atan2(point2[1] - point1[1], point2[0] - point1[0]))
            return heading

        def closest_2_racing_points_index(racing_coords, car_coords):

            # Calculate all distances to racing points
            distances = []
            for i in range(len(racing_coords)):
                distance = dist_2_points(x1=racing_coords[i][0], x2=car_coords[0],
                                         y1=racing_coords[i][1], y2=car_coords[1])
                distances.append(distance)

            # Get index of the closest racing point
            closest_index = distances.index(min(distances))

            # Get index of the second closest racing point
            distances_no_closest = distances.copy()
            distances_no_closest[closest_index] = 999
            second_closest_index = distances_no_closest.index(
                min(distances_no_closest))

            return [closest_index, second_closest_index]

        def dist_to_racing_line(closest_coords, second_closest_coords, car_coords):

            # Calculate the distances between 2 closest racing points
            a = abs(dist_2_points(x1=closest_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=closest_coords[1],
                                  y2=second_closest_coords[1]))

            # Distances between car and closest and second closest racing point
            b = abs(dist_2_points(x1=car_coords[0],
                                  x2=closest_coords[0],
                                  y1=car_coords[1],
                                  y2=closest_coords[1]))
            c = abs(dist_2_points(x1=car_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=car_coords[1],
                                  y2=second_closest_coords[1]))

            # Calculate distance between car and racing line (goes through 2 closest racing points)
            # try-except in case a=0 (rare bug in DeepRacer)
            try:
                distance = abs(-(a**4) + 2*(a**2)*(b**2) + 2*(a**2)*(c**2) -
                                (b**4) + 2*(b**2)*(c**2) - (c**4))**0.5 / (2*a)
            except:
                distance = b

            return distance

        # Calculate which one of the closest racing points is the next one and which one the previous one
        def next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading):

            # Virtually set the car more into the heading direction
            heading_vector = [math.cos(math.radians(
                heading)), math.sin(math.radians(heading))]
            new_car_coords = [car_coords[0]+heading_vector[0],
                              car_coords[1]+heading_vector[1]]

            # Calculate distance from new car coords to 2 closest racing points
            distance_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                        x2=closest_coords[0],
                                                        y1=new_car_coords[1],
                                                        y2=closest_coords[1])
            distance_second_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                               x2=second_closest_coords[0],
                                                               y1=new_car_coords[1],
                                                               y2=second_closest_coords[1])

            if distance_closest_coords_new <= distance_second_closest_coords_new:
                next_point_coords = closest_coords
                prev_point_coords = second_closest_coords
            else:
                next_point_coords = second_closest_coords
                prev_point_coords = closest_coords

            return [next_point_coords, prev_point_coords]

        def racing_direction_diff(closest_coords, second_closest_coords, car_coords, heading):

            # Calculate the direction of the center line based on the closest waypoints
            next_point, prev_point = next_prev_racing_point(closest_coords,
                                                            second_closest_coords,
                                                            car_coords,
                                                            heading)

            # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
            track_direction = math.atan2(
                next_point[1] - prev_point[1], next_point[0] - prev_point[0])

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
            current_actual_time = (step_count-1) / 15

            # Calculate which indexes were already passed
            indexes_traveled = indexes_cyclical(
                first_index, closest_index, len(times_list))

            # Calculate how much time should have passed if car would have followed optimals
            current_expected_time = sum(
                [times_list[i] for i in indexes_traveled])

            # Calculate how long one entire lap takes if car follows optimals
            total_expected_time = sum(times_list)

            # Calculate how long car would take for entire lap, if it continued like it did until now
            try:
                projected_time = (current_actual_time /
                                  current_expected_time) * total_expected_time
            except:
                projected_time = 9999

            return projected_time

        #################### RACING LINE ######################

        # Optimal racing line for the 2022_may_open_pro_cw track
        # Each row: [x,y,speed,timeFromPreviousPoint]
        racing_track = [[-0.20866, -5.36317, 4.0, 0.06756],
                        [-0.47948, -5.37029, 4.0, 0.06773],
                        [-0.75123, -5.37617, 4.0, 0.06795],
                        [-1.02409, -5.38103, 4.0, 0.06822],
                        [-1.29812, -5.38506, 4.0, 0.06852],
                        [-1.57322, -5.38848, 4.0, 0.06878],
                        [-1.84898, -5.3916, 4.0, 0.06894],
                        [-2.12539, -5.39445, 4.0, 0.06911],
                        [-2.40259, -5.39703, 4.0, 0.0693],
                        [-2.68069, -5.39937, 4.0, 0.06953],
                        [-2.95977, -5.40149, 3.77462, 0.07394],
                        [-3.2402, -5.40335, 3.19514, 0.08777],
                        [-3.52371, -5.40475, 2.77935, 0.10201],
                        [-3.82523, -5.40512, 2.3749, 0.12696],
                        [-4.1249, -5.40147, 2.07694, 0.1443],
                        [-4.42006, -5.38893, 1.83175, 0.16128],
                        [-4.70782, -5.36302, 1.61975, 0.17838],
                        [-4.98506, -5.31993, 1.30909, 0.21432],
                        [-5.24833, -5.25647, 1.30909, 0.20687],
                        [-5.49259, -5.16867, 1.30909, 0.19828],
                        [-5.71233, -5.05402, 1.30909, 0.18933],
                        [-5.90103, -4.91127, 1.30909, 0.18075],
                        [-6.05035, -4.7407, 1.30909, 0.17317],
                        [-6.13856, -4.54291, 1.41729, 0.15281],
                        [-6.17498, -4.33455, 1.47262, 0.14363],
                        [-6.16515, -4.12459, 1.47262, 0.14273],
                        [-6.11324, -3.91927, 1.47262, 0.14382],
                        [-6.014, -3.72679, 1.63841, 0.13218],
                        [-5.87623, -3.55121, 1.79294, 0.12447],
                        [-5.70559, -3.39472, 1.96297, 0.11795],
                        [-5.5067, -3.25851, 2.12925, 0.11321],
                        [-5.28316, -3.14351, 2.33107, 0.10784],
                        [-5.0387, -3.04978, 2.59171, 0.10102],
                        [-4.77737, -2.97619, 2.92726, 0.09275],
                        [-4.50311, -2.92053, 3.33261, 0.08397],
                        [-4.21931, -2.88014, 3.84397, 0.07458],
                        [-3.92871, -2.85209, 4.0, 0.07299],
                        [-3.63343, -2.83368, 4.0, 0.07396],
                        [-3.33495, -2.8226, 4.0, 0.07467],
                        [-3.03458, -2.81638, 4.0, 0.07511],
                        [-2.73329, -2.81274, 4.0, 0.07533],
                        [-2.43178, -2.80973, 4.0, 0.07538],
                        [-2.13028, -2.80671, 4.0, 0.07538],
                        [-1.82877, -2.80369, 4.0, 0.07538],
                        [-1.52727, -2.80067, 4.0, 0.07538],
                        [-1.22577, -2.79765, 4.0, 0.07538],
                        [-0.92426, -2.79464, 3.98664, 0.07563],
                        [-0.62276, -2.79161, 3.52643, 0.0855],
                        [-0.32186, -2.78711, 3.1538, 0.09542],
                        [-0.02262, -2.77866, 2.84677, 0.10516],
                        [0.27376, -2.76377, 2.56939, 0.1155],
                        [0.56579, -2.73973, 2.24788, 0.13035],
                        [0.85179, -2.70399, 2.00139, 0.14401],
                        [1.12993, -2.65416, 1.741, 0.1623],
                        [1.39813, -2.58792, 1.741, 0.15868],
                        [1.65411, -2.50313, 1.741, 0.15488],
                        [1.89508, -2.39755, 1.741, 0.15111],
                        [2.11637, -2.26737, 1.741, 0.14747],
                        [2.31239, -2.10939, 1.741, 0.1446],
                        [2.47312, -1.91918, 1.87265, 0.13298],
                        [2.60078, -1.70495, 2.00684, 0.12427],
                        [2.69723, -1.47288, 2.16346, 0.11617],
                        [2.76475, -1.22824, 2.28557, 0.11104],
                        [2.80472, -0.97508, 2.40069, 0.10676],
                        [2.81854, -0.71683, 2.42202, 0.10678],
                        [2.80775, -0.45641, 2.42202, 0.10761],
                        [2.7738, -0.19625, 2.42202, 0.10832],
                        [2.71744, 0.06147, 2.42202, 0.10892],
                        [2.63958, 0.31476, 2.42202, 0.10941],
                        [2.53846, 0.56071, 2.42202, 0.10979],
                        [2.41213, 0.79488, 2.32443, 0.11447],
                        [2.26679, 1.017, 2.16525, 0.1226],
                        [2.10441, 1.22556, 1.93268, 0.13676],
                        [1.92622, 1.41833, 1.735, 0.1513],
                        [1.73373, 1.59297, 1.5189, 0.17111],
                        [1.52843, 1.74653, 1.5189, 0.16879],
                        [1.3117, 1.87457, 1.5189, 0.16573],
                        [1.08586, 1.97299, 1.5189, 0.16219],
                        [0.8534, 2.03495, 1.5189, 0.15839],
                        [0.61805, 2.05323, 1.5189, 0.15541],
                        [0.3853, 2.01592, 1.60347, 0.14701],
                        [0.15996, 1.92756, 1.88066, 0.1287],
                        [-0.05666, 1.79918, 2.34534, 0.10736],
                        [-0.2658, 1.6426, 3.41065, 0.0766],
                        [-0.47081, 1.47166, 4.0, 0.06673],
                        [-0.69733, 1.29632, 4.0, 0.07161],
                        [-0.92948, 1.12892, 4.0, 0.07155],
                        [-1.16615, 0.96902, 4.0, 0.07141],
                        [-1.40667, 0.81631, 4.0, 0.07123],
                        [-1.65068, 0.67079, 4.0, 0.07103],
                        [-1.89787, 0.53243, 4.0, 0.07082],
                        [-2.14816, 0.40173, 4.0, 0.07059],
                        [-2.40152, 0.27937, 4.0, 0.07034],
                        [-2.65806, 0.16665, 3.85053, 0.07277],
                        [-2.91746, 0.06387, 3.60635, 0.07737],
                        [-3.17928, -0.02927, 3.33042, 0.08344],
                        [-3.44319, -0.11276, 3.04357, 0.09094],
                        [-3.70888, -0.18617, 2.74985, 0.10024],
                        [-3.97601, -0.24885, 2.45359, 0.11183],
                        [-4.24417, -0.29982, 2.1883, 0.12473],
                        [-4.51275, -0.33756, 1.90582, 0.14231],
                        [-4.7809, -0.35997, 1.67875, 0.16028],
                        [-5.04721, -0.36441, 1.48594, 0.17925],
                        [-5.30954, -0.34758, 1.3, 0.20221],
                        [-5.56448, -0.30538, 1.3, 0.19877],
                        [-5.80705, -0.23371, 1.3, 0.19457],
                        [-6.02934, -0.12741, 1.3, 0.18953],
                        [-6.22076, 0.01671, 1.3, 0.18432],
                        [-6.36743, 0.19907, 1.3, 0.18002],
                        [-6.44818, 0.41385, 1.54124, 0.14887],
                        [-6.4793, 0.63937, 1.66755, 0.13653],
                        [-6.46738, 0.86724, 1.7771, 0.1284],
                        [-6.41711, 1.09209, 1.90321, 0.12106],
                        [-6.33297, 1.31032, 2.03861, 0.11473],
                        [-6.21886, 1.5196, 2.06992, 0.11516],
                        [-6.07778, 1.71833, 2.06992, 0.11774],
                        [-5.9065, 1.90193, 2.31981, 0.10824],
                        [-5.71079, 2.07045, 2.64397, 0.09768],
                        [-5.4959, 2.22532, 3.09621, 0.08555],
                        [-5.26673, 2.36902, 3.82178, 0.07078],
                        [-5.02797, 2.50478, 4.0, 0.06866],
                        [-4.78438, 2.63642, 4.0, 0.06922],
                        [-4.53202, 2.77652, 4.0, 0.07216],
                        [-4.28081, 2.91917, 4.0, 0.07222],
                        [-4.0306, 3.06401, 4.0, 0.07228],
                        [-3.78129, 3.21079, 4.0, 0.07233],
                        [-3.53277, 3.35926, 4.0, 0.07237],
                        [-3.28498, 3.50928, 4.0, 0.07242],
                        [-3.03787, 3.66073, 4.0, 0.07246],
                        [-2.79141, 3.81355, 4.0, 0.0725],
                        [-2.54562, 3.96777, 3.60451, 0.0805],
                        [-2.30712, 4.11952, 3.14211, 0.08997],
                        [-2.06799, 4.26909, 2.73637, 0.10308],
                        [-1.82752, 4.4141, 2.4628, 0.11402],
                        [-1.58498, 4.55197, 2.4628, 0.11328],
                        [-1.3396, 4.67996, 2.4628, 0.11238],
                        [-1.09047, 4.79478, 2.4628, 0.11138],
                        [-0.83665, 4.89264, 2.4628, 0.11046],
                        [-0.57703, 4.96853, 2.4628, 0.10983],
                        [-0.31082, 5.01736, 3.07913, 0.0879],
                        [-0.04065, 5.04867, 3.51291, 0.07742],
                        [0.23262, 5.06628, 4.0, 0.06846],
                        [0.50845, 5.07334, 4.0, 0.06898],
                        [0.78637, 5.07292, 4.0, 0.06948],
                        [1.06574, 5.06805, 4.0, 0.06986],
                        [1.34901, 5.06525, 4.0, 0.07082],
                        [1.63229, 5.06415, 4.0, 0.07082],
                        [1.91557, 5.06429, 4.0, 0.07082],
                        [2.19886, 5.06535, 3.58111, 0.07911],
                        [2.48215, 5.06702, 3.13848, 0.09027],
                        [2.75795, 5.06666, 2.78929, 0.09888],
                        [3.03093, 5.06214, 2.42834, 0.11243],
                        [3.30007, 5.05086, 2.14518, 0.12557],
                        [3.5647, 5.03029, 1.93316, 0.1373],
                        [3.82399, 4.99773, 1.73594, 0.15054],
                        [4.07675, 4.95025, 1.73594, 0.14815],
                        [4.32126, 4.88485, 1.73594, 0.1458],
                        [4.5546, 4.79711, 1.73594, 0.14361],
                        [4.77277, 4.68239, 1.73594, 0.14199],
                        [4.97045, 4.53638, 1.73594, 0.14157],
                        [5.13887, 4.35384, 1.80855, 0.13733],
                        [5.27552, 4.13957, 2.1117, 0.12034],
                        [5.38528, 3.90298, 2.4076, 0.10833],
                        [5.47184, 3.64954, 2.74081, 0.09772],
                        [5.53865, 3.3832, 3.14105, 0.08742],
                        [5.58913, 3.10714, 3.72737, 0.07529],
                        [5.62723, 2.8242, 4.0, 0.07137],
                        [5.65712, 2.53692, 4.0, 0.07221],
                        [5.68233, 2.24642, 4.0, 0.0729],
                        [5.71226, 1.95247, 4.0, 0.07387],
                        [5.74597, 1.65878, 4.0, 0.0739],
                        [5.78293, 1.36571, 4.0, 0.07385],
                        [5.82274, 1.07328, 4.0, 0.07378],
                        [5.86513, 0.7815, 4.0, 0.07371],
                        [5.90991, 0.49034, 4.0, 0.07364],
                        [5.95689, 0.19981, 4.0, 0.07358],
                        [6.00599, -0.09011, 4.0, 0.07351],
                        [6.05726, -0.37936, 4.0, 0.07344],
                        [6.1108, -0.66787, 3.5934, 0.08166],
                        [6.16684, -0.95552, 2.83539, 0.10336],
                        [6.22576, -1.24209, 2.396, 0.1221],
                        [6.28809, -1.52722, 2.06211, 0.14154],
                        [6.35106, -1.79549, 1.67917, 0.1641],
                        [6.4071, -2.06285, 1.67917, 0.16268],
                        [6.44946, -2.3284, 1.67917, 0.16014],
                        [6.47098, -2.59111, 1.67917, 0.15698],
                        [6.46496, -2.84973, 1.67917, 0.15406],
                        [6.42349, -3.10203, 1.67917, 0.15227],
                        [6.33019, -3.34062, 1.86179, 0.1376],
                        [6.19601, -3.56223, 2.07578, 0.12481],
                        [6.03027, -3.76546, 2.29148, 0.11444],
                        [5.84, -3.95021, 2.50756, 0.10577],
                        [5.63044, -4.11721, 2.72747, 0.09825],
                        [5.40555, -4.26756, 2.95373, 0.09159],
                        [5.16839, -4.40254, 3.18883, 0.08557],
                        [4.9214, -4.52346, 3.43491, 0.08006],
                        [4.66658, -4.63159, 3.7428, 0.07396],
                        [4.40575, -4.72845, 3.99229, 0.06969],
                        [4.14024, -4.81503, 4.0, 0.06982],
                        [3.87116, -4.89222, 4.0, 0.06998],
                        [3.59952, -4.96082, 4.0, 0.07004],
                        [3.32617, -5.02154, 4.0, 0.07],
                        [3.05186, -5.0751, 4.0, 0.06987],
                        [2.77721, -5.12214, 4.0, 0.06966],
                        [2.50269, -5.16329, 4.0, 0.06939],
                        [2.22868, -5.19918, 4.0, 0.06909],
                        [1.9554, -5.23038, 4.0, 0.06876],
                        [1.68296, -5.25741, 4.0, 0.06844],
                        [1.4114, -5.28064, 4.0, 0.06814],
                        [1.14061, -5.30063, 4.0, 0.06788],
                        [0.87045, -5.31767, 4.0, 0.06767],
                        [0.60072, -5.33212, 4.0, 0.06753],
                        [0.33115, -5.34431, 4.0, 0.06746],
                        [0.06145, -5.35455, 4.0, 0.06747]]

        ################## INPUT PARAMETERS ###################

        # Read all input parameters
        all_wheels_on_track = params['all_wheels_on_track']
        x = params['x']
        y = params['y']
        distance_from_center = params['distance_from_center']
        is_left_of_center = params['is_left_of_center']
        heading = params['heading']
        progress = params['progress']
        steps = params['steps']
        speed = params['speed']
        steering_angle = params['steering_angle']
        track_width = params['track_width']
        waypoints = params['waypoints']
        closest_waypoints = params['closest_waypoints']
        is_offtrack = params['is_offtrack']
        abs_steering_angle = abs(params['steering_angle'])
        ############### OPTIMAL X,Y,SPEED,TIME ################

        # Get closest indexes for racing line (and distances to all points on racing line)
        closest_index, second_closest_index = closest_2_racing_points_index(
            racing_track, [x, y])

        # Get optimal [x, y, speed, time] for closest and second closest index
        optimals = racing_track[closest_index]
        optimals_second = racing_track[second_closest_index]

        # Save first racingpoint of episode for later
        if self.verbose == True:
            self.first_racingpoint_index = 0  # this is just for testing purposes
        if steps == 1:
            self.first_racingpoint_index = closest_index

        ################ REWARD AND PUNISHMENT ################

        ## Define the default reward ##
        reward = 1
        
        ## Reward if car goes close to optimal racing line ##
        DISTANCE_MULTIPLE = 1 # inreased reward, but beware of speed sacrifice 
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(1e-3, 1 - (dist/(track_width*0.25))) # tweaked from 0.3 to 0.25
        reward += distance_reward * DISTANCE_MULTIPLE
        
        ## Reward if speed is close to optimal speed
        SPEED_DIFF_NO_REWARD = 0.25  # Maximum m/s difference to optimal speed
        SPEED_MULTIPLE = 3  # Maximum reward multiplier
        speed_diff = abs(optimals[2] - speed)
        
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            # Calculate the bell curve reward based on speed difference
            bell_curve_multiplier = math.exp(-(speed_diff ** 2) / (2 * (SPEED_DIFF_NO_REWARD ** 2)))
            speed_reward = 1 + (SPEED_MULTIPLE - 1) * bell_curve_multiplier
        else:
            speed_reward = 1
            
        reward += speed_reward

        # Reward if less steps
        REWARD_PER_STEP_FOR_FASTEST_TIME = 1 # 8.7.23 adjusted back to 1
        STANDARD_TIME = 22
        FASTEST_TIME = 20
        times_list = [row[3] for row in racing_track]
        projected_time = projected_time(
            self.first_racingpoint_index, closest_index, steps, times_list)
        try:
            steps_prediction = projected_time * 15 + 1
            reward_prediction = max(1e-3, (-REWARD_PER_STEP_FOR_FASTEST_TIME*(FASTEST_TIME) /
                                           (STANDARD_TIME-FASTEST_TIME))*(steps_prediction-(STANDARD_TIME*15+1)))
            steps_reward = min(REWARD_PER_STEP_FOR_FASTEST_TIME,
                               reward_prediction / steps_prediction)
        except:
            steps_reward = 0
        reward += steps_reward

        # Zero reward if obviously wrong direction (e.g. spin)
        direction_diff = racing_direction_diff(
            optimals[0:2], optimals_second[0:2], [x, y], heading)
        if direction_diff > 30:
            reward = 1e-3

        # Zero reward of obviously too slow
        speed_diff_zero = optimals[2]-speed
        if speed_diff_zero > 0.25: #8.6.23 tweaked to 0.25
            reward *= 0.5 #50% reward penalty

        # Steering penality threshold, change the number based on your action space setting
        ABS_STEERING_THRESHOLD = 20 # maximum 20 degrees steering threshold

        # Penalize reward if the car is steering too much
        if steering_angle > ABS_STEERING_THRESHOLD:
            reward *= 0.9

        ################### V4 New Reward Component - Look Ahead #########################
        # Check if the car is on track
        if all_wheels_on_track and not is_offtrack:

            # Number of waypoints to look ahead
            LOOKAHEAD_POINTS = 10  # Adjust as needed

            # Get the current closest waypoint index
            next_waypoint_index = closest_waypoints[1]

            # Initialize variables to store the maximum angle difference
            max_angle_diff = 0.0

            # Loop through the next N waypoints
            for i in range(1, LOOKAHEAD_POINTS + 1):
                # Calculate the index of the next waypoint, accounting for track looping
                waypoint_index = (next_waypoint_index + i) % len(waypoints)
                prev_waypoint_index = (waypoint_index - 1) % len(waypoints)

                # Get the coordinates of the waypoints
                point_prev = waypoints[prev_waypoint_index]
                point_next = waypoints[waypoint_index]

                # Calculate the heading between the waypoints
                waypoint_heading = calculate_heading(point_prev, point_next)

                # Calculate the difference between this heading and the previous heading
                if i == 1:
                    # For the first iteration, use the car's current heading
                    heading_prev = heading
                else:
                    heading_prev = prev_waypoint_heading

                angle_diff = abs(waypoint_heading - heading_prev)
                # Adjust angle difference to be between 0 and 180 degrees
                if angle_diff > 180:
                    angle_diff = 360 - angle_diff

                # Update the maximum angle difference
                if angle_diff > max_angle_diff:
                    max_angle_diff = angle_diff

                # Store the previous waypoint heading for next iteration
                prev_waypoint_heading = waypoint_heading

            # If the maximum angle difference is less than or equal to 5 degrees
            if max_angle_diff <= 5.0:
                # Penalize steering angles greater than 5 degrees
                STEERING_THRESHOLD = 5.0  # degrees
                if abs_steering_angle > STEERING_THRESHOLD:
                    steering_penalty = 0.8  # Penalize by reducing reward
                    reward *= steering_penalty
                    if self.verbose:
                        print(f"Steering penalty applied. Steering angle: {steering_angle}")

                # Reward higher speeds
                SPEED_THRESHOLD = optimals[2] + 1.0  # Encourage speed higher than optimal + 1 m/s
                if speed > SPEED_THRESHOLD:
                    speed_bonus = 1.2  # Increase reward
                    reward *= speed_bonus
                    if self.verbose:
                        print(f"Speed bonus applied. Speed: {speed}")

        ############################################

        ## Incentive for finishing the lap in less steps ## 091624 Adjusted
        LEAST_STEPS = FASTEST_TIME * 15
        if progress == 100:
            # Calculate the difference in steps
            difference = steps - LEAST_STEPS
            # Define maximum allowed difference to prevent huge rewards/penalties
            MAX_DIFF = 100  # Adjust based on expected step ranges
            # Bound the difference within [-MAX_DIFF, MAX_DIFF]
            difference = max(min(difference, MAX_DIFF), -MAX_DIFF)
            # Normalize the difference to [-1, 1]
            normalized_diff = difference / MAX_DIFF
            # Calculate the cubic reward component
            cubic_component = normalized_diff ** 3
            # Adjust the reward
            reward += 50 - (50 * cubic_component)

        ## Zero reward if off track ##
        if not all_wheels_on_track or is_offtrack:
            reward = 1e-3

        finish_reward = reward
            
        ####################### VERBOSE #######################

        if self.verbose == True:
            print("------------------")
            print("Closest index: %i" % closest_index)
            print("Distance to racing line: %f" % dist)
            print("=== Distance reward (w/out multiple): %f ===" %
                  (distance_reward))
            print("Optimal speed: %f" % optimals[2])
            print("Speed difference: %f" % speed_diff)
            print("=== Speed reward (w/out multiple): %f ===" % speed_reward)
            print("Direction difference: %f" % direction_diff)
            print("Predicted time: %f" % projected_time)
            print("=== Steps reward: %f ===" % steps_reward)
            print("=== Finish reward: %f ===" % finish_reward)
            print("Progress: %f" % progress)
            print("Steps: %i" % steps)
            print("Speed: %f" % speed)
            print("Steering angle: %f" % steering_angle)
            print("------------------")
            if progress == 100:
                print("Reward for finishing the lap: %f" % finish_reward)

        #################### RETURN REWARD ####################

        # Always return a float value
        return float(reward)


reward_object = Reward(verbose=True)  # add parameter verbose=True to get noisy output for testing


def reward_function(params):
    return reward_object.reward_function(params)