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

        # Optimal racing line for the 2022_may_open_ccw track
        # Each row: [x,y,speed,timeFromPreviousPoint]
        racing_track = [[5.03664, 0.67231, 4.0, 0.05965],
                        [5.03045, 0.90704, 4.0, 0.0587],
                        [5.02249, 1.14035, 4.0, 0.05836],
                        [5.01246, 1.37425, 3.82187, 0.06126],
                        [4.99972, 1.61082, 3.38127, 0.07007],
                        [4.98351, 1.85224, 2.99627, 0.08076],
                        [4.96266, 2.10072, 2.61781, 0.09525],
                        [4.93558, 2.35785, 2.2816, 0.11332],
                        [4.89998, 2.62368, 2.01343, 0.13321],
                        [4.85329, 2.89485, 1.76693, 0.15573],
                        [4.79306, 3.16539, 1.76693, 0.15686],
                        [4.71681, 3.42962, 1.76693, 0.15565],
                        [4.62114, 3.683, 1.76693, 0.15329],
                        [4.50173, 3.92112, 1.76693, 0.15076],
                        [4.35416, 4.13861, 1.76693, 0.14875],
                        [4.17335, 4.32692, 1.93754, 0.13474],
                        [3.96733, 4.48865, 2.18171, 0.12005],
                        [3.74254, 4.62781, 2.41007, 0.1097],
                        [3.50295, 4.74717, 2.64501, 0.1012],
                        [3.25145, 4.84901, 2.87759, 0.09429],
                        [2.99024, 4.9351, 3.1078, 0.0885],
                        [2.72114, 5.0069, 3.33226, 0.08358],
                        [2.44576, 5.0656, 3.55097, 0.07929],
                        [2.16556, 5.11223, 3.78059, 0.07513],
                        [1.8819, 5.14783, 3.96933, 0.07202],
                        [1.59589, 5.17314, 4.0, 0.07178],
                        [1.30849, 5.1889, 4.0, 0.07196],
                        [1.02048, 5.19563, 4.0, 0.07202],
                        [0.73251, 5.19373, 4.0, 0.07199],
                        [0.44511, 5.18365, 4.0, 0.07189],
                        [0.1587, 5.16587, 3.97701, 0.07215],
                        [-0.12638, 5.14081, 3.6471, 0.07847],
                        [-0.40988, 5.10893, 3.28763, 0.08678],
                        [-0.69145, 5.07001, 2.9581, 0.09609],
                        [-0.9706, 5.02359, 2.86043, 0.09893],
                        [-1.24668, 4.96886, 2.57192, 0.10943],
                        [-1.5188, 4.90476, 2.51155, 0.11131],
                        [-1.78576, 4.82988, 2.51155, 0.11039],
                        [-2.04581, 4.74233, 2.29458, 0.11959],
                        [-2.29668, 4.63996, 2.06581, 0.13116],
                        [-2.53711, 4.52275, 1.80163, 0.14847],
                        [-2.76391, 4.38854, 1.5948, 0.16525],
                        [-2.97563, 4.23811, 1.56468, 0.16599],
                        [-3.17185, 4.07312, 1.56468, 0.16385],
                        [-3.34873, 3.89231, 1.56468, 0.16166],
                        [-3.50093, 3.69455, 1.56468, 0.15948],
                        [-3.61934, 3.47899, 1.40533, 0.17501],
                        [-3.69292, 3.24865, 1.40533, 0.17207],
                        [-3.71948, 3.0125, 1.40533, 0.16909],
                        [-3.71398, 2.77708, 1.40533, 0.16757],
                        [-3.67209, 2.54612, 1.40533, 0.16703],
                        [-3.58613, 2.32618, 1.40533, 0.16803],
                        [-3.44709, 2.12974, 1.66051, 0.14493],
                        [-3.2716, 1.95714, 1.85951, 0.13237],
                        [-3.06756, 1.80778, 2.06306, 0.12257],
                        [-2.84044, 1.68056, 2.32031, 0.11219],
                        [-2.59509, 1.57335, 2.61133, 0.10253],
                        [-2.33539, 1.48383, 3.01217, 0.0912],
                        [-2.06496, 1.40887, 3.44851, 0.08138],
                        [-1.78699, 1.34498, 3.0311, 0.0941],
                        [-1.50428, 1.28847, 2.70773, 0.10647],
                        [-1.22109, 1.23663, 2.40105, 0.1199],
                        [-0.94256, 1.17855, 2.13262, 0.13341],
                        [-0.67063, 1.11166, 1.8881, 0.14832],
                        [-0.40723, 1.03361, 1.63868, 0.16764],
                        [-0.15464, 0.94211, 1.63868, 0.16395],
                        [0.08465, 0.83515, 1.63868, 0.15995],
                        [0.30718, 0.71051, 1.49142, 0.17102],
                        [0.50866, 0.56612, 1.3, 0.19067],
                        [0.68335, 0.40016, 1.3, 0.18535],
                        [0.82197, 0.21117, 1.3, 0.18029],
                        [0.92921, 0.00706, 1.3, 0.17736],
                        [0.99958, -0.21008, 1.3, 0.17558],
                        [1.02388, -0.43634, 1.3, 0.17505],
                        [0.98664, -0.66071, 1.4506, 0.15679],
                        [0.90153, -0.87221, 1.61125, 0.14149],
                        [0.77906, -1.06743, 1.71909, 0.13406],
                        [0.62441, -1.24418, 1.74142, 0.13486],
                        [0.43879, -1.39862, 1.92319, 0.12555],
                        [0.22791, -1.53098, 2.14273, 0.1162],
                        [-0.00339, -1.64263, 2.34858, 0.10936],
                        [-0.25178, -1.7346, 2.60767, 0.10157],
                        [-0.51402, -1.80862, 2.94727, 0.09245],
                        [-0.78696, -1.86715, 3.27759, 0.08517],
                        [-1.06837, -1.91212, 3.59041, 0.07937],
                        [-1.35612, -1.94587, 3.25588, 0.08898],
                        [-1.64862, -1.97041, 2.92435, 0.10037],
                        [-1.94487, -1.98661, 2.64343, 0.11224],
                        [-2.23701, -2.01148, 2.36299, 0.12408],
                        [-2.52387, -2.04664, 2.14978, 0.13444],
                        [-2.80406, -2.09377, 1.87544, 0.1515],
                        [-3.07599, -2.15458, 1.62023, 0.17198],
                        [-3.33753, -2.23116, 1.62023, 0.1682],
                        [-3.58624, -2.32544, 1.62023, 0.16416],
                        [-3.81873, -2.43973, 1.62023, 0.15989],
                        [-4.03138, -2.5755, 1.54658, 0.16313],
                        [-4.21742, -2.7356, 1.54658, 0.1587],
                        [-4.36628, -2.92171, 1.54658, 0.1541],
                        [-4.48254, -3.12444, 1.54658, 0.15111],
                        [-4.5644, -3.33985, 1.54351, 0.1493],
                        [-4.60903, -3.5638, 1.54351, 0.14794],
                        [-4.61016, -3.79083, 1.54351, 0.14708],
                        [-4.57484, -4.01497, 1.54351, 0.14701],
                        [-4.50302, -4.23161, 1.54351, 0.14786],
                        [-4.39303, -4.435, 1.54351, 0.14981],
                        [-4.24192, -4.61681, 1.66783, 0.14175],
                        [-4.05675, -4.77401, 1.87053, 0.12985],
                        [-3.84505, -4.90736, 2.0621, 0.12133],
                        [-3.61164, -5.01776, 2.27247, 0.11362],
                        [-3.36045, -5.10651, 2.5331, 0.10517],
                        [-3.09509, -5.17566, 2.82873, 0.09694],
                        [-2.8186, -5.22743, 3.18861, 0.08822],
                        [-2.53365, -5.26435, 3.67407, 0.0782],
                        [-2.24262, -5.28935, 4.0, 0.07303],
                        [-1.94729, -5.30503, 4.0, 0.07394],
                        [-1.64895, -5.3135, 4.0, 0.07462],
                        [-1.34872, -5.317, 4.0, 0.07506],
                        [-1.04746, -5.31762, 4.0, 0.07531],
                        [-0.74582, -5.31704, 4.0, 0.07541],
                        [-0.44443, -5.31581, 4.0, 0.07535],
                        [-0.14349, -5.31345, 4.0, 0.07524],
                        [0.15679, -5.30944, 4.0, 0.07508],
                        [0.45618, -5.30325, 4.0, 0.07486],
                        [0.75441, -5.29432, 4.0, 0.07459],
                        [1.05122, -5.28207, 3.90683, 0.07604],
                        [1.34626, -5.26581, 3.50424, 0.08432],
                        [1.63915, -5.24479, 3.1395, 0.09353],
                        [1.92937, -5.21801, 2.75339, 0.10585],
                        [2.21637, -5.18455, 2.43801, 0.11852],
                        [2.49924, -5.14279, 2.14742, 0.13315],
                        [2.77695, -5.09112, 1.90779, 0.14806],
                        [3.0482, -5.02763, 1.75791, 0.15847],
                        [3.31135, -4.95015, 1.75791, 0.15605],
                        [3.56388, -4.85553, 1.75791, 0.1534],
                        [3.80264, -4.74051, 1.75791, 0.15076],
                        [4.02317, -4.60111, 1.75791, 0.14841],
                        [4.21949, -4.43331, 1.75791, 0.14691],
                        [4.38502, -4.23491, 2.02469, 0.12762],
                        [4.52531, -4.01501, 2.22792, 0.11708],
                        [4.64317, -3.77795, 2.47665, 0.1069],
                        [4.74151, -3.52724, 2.70819, 0.09944],
                        [4.82235, -3.26521, 2.92024, 0.0939],
                        [4.88711, -2.99355, 3.17741, 0.08789],
                        [4.9374, -2.71386, 3.51042, 0.08095],
                        [4.97515, -2.42775, 3.9382, 0.07328],
                        [5.00244, -2.13671, 4.0, 0.07308],
                        [5.0214, -1.84204, 4.0, 0.07382],
                        [5.03388, -1.54476, 4.0, 0.07439],
                        [5.04148, -1.24566, 4.0, 0.0748],
                        [5.04557, -0.94532, 4.0, 0.07509],
                        [5.04734, -0.64423, 4.0, 0.07528],
                        [5.04771, -0.3443, 4.0, 0.07498],
                        [5.04676, -0.06848, 4.0, 0.06896],
                        [5.04463, 0.18831, 4.0, 0.0642],
                        [5.0413, 0.43375, 4.0, 0.06137]]

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
        ## TODO: Initialize the reward at 1 for the first step, however, if progress increases, reward increases. 
        
        reward = 1
        reward_multiple = 1e-3

        # Steering penality threshold, change the number based on your action space setting
        ABS_STEERING_THRESHOLD = 30 # maximum 30 degrees steering threshold

        ## Reward if car goes close to optimal racing line ##
        DISTANCE_MULTIPLE = 1 # inreased reward, but beware of speed sacrifice 
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(1e-3, 1 - (dist/(track_width*0.25))) # Reward Cap = 1, if dist is less than 0.25 track_width, reward is 1

        ## Reward if speed is close to optimal speed; max speed = 4 
        SPEED_MULTIPLE = 1
        speed_diff = (optimals[2]-speed) / (optimals[2] + 1e-3)
        speed_reward = (1 - speed_diff) * SPEED_MULTIPLE
        
        # Zero reward if obviously wrong direction (e.g. spin)
        direction_diff = racing_direction_diff(
            optimals[0:2], optimals_second[0:2], [x, y], heading)
        DIRECTION_DIFF_THRESHOLD = 20

        # Reward if less steps
        REWARD_PER_STEP_FOR_FASTEST_TIME = 1 # 8.7.23 adjusted back to 1
        STANDARD_TIME = 25
        FASTEST_TIME = 19
        times_list = [row[3] for row in racing_track]
        projected_time = projected_time(
            self.first_racingpoint_index, closest_index, steps, times_list)
        steps_prediction = projected_time * 15 + 1
        reward_prediction = max(1e-3, (-REWARD_PER_STEP_FOR_FASTEST_TIME*(FASTEST_TIME) /
                                    (STANDARD_TIME-FASTEST_TIME))*(steps_prediction-(STANDARD_TIME*15+1)))
        try:
            steps_reward = min(REWARD_PER_STEP_FOR_FASTEST_TIME,
                            reward_prediction / steps_prediction)
        except:
            steps_reward = 0        

        if direction_diff < DIRECTION_DIFF_THRESHOLD and not steering_angle > ABS_STEERING_THRESHOLD and not is_offtrack and all_wheels_on_track and speed > optimals[2]:
            reward_multiple += max(1e-3, distance_reward * DISTANCE_MULTIPLE) * max(1e-3, speed_reward) * max(1e-3, steps_reward)
        else:
            reward = 1e-3

        ## Incentive for finishing the lap in less steps ##
        # should be adapted to track length and other rewards
        REWARD_FOR_FASTEST_TIME = 1500
        finish_reward = 1e-3

        if progress == 100:
            finish_reward = max(1e-3, (-REWARD_FOR_FASTEST_TIME /
                                       (15*(STANDARD_TIME-FASTEST_TIME)))*(steps-STANDARD_TIME*15))
        else:
            finish_reward = 1e-3

        reward += finish_reward

        ## Incentive for finishing the lap in less steps at the end ## 091624 Adjusted
        LEAST_STEPS = FASTEST_TIME*15 # This is the fastest estimated time
        if progress == 100:
            steps_multiple = LEAST_STEPS / steps
            reward *= steps_multiple 

        final_reward = reward * reward_multiple
            
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
            print("=== Reward multiple: %f ===" % reward_multiple)
            print("=== Finish reward: %f ===" % finish_reward)
            print("=== Final reward: %f ===" % final_reward)
            print("Progress: %f" % progress)
            print("Steps: %i" % steps)
            print("Speed: %f" % speed)
            print("Steering angle: %f" % steering_angle)
            print("------------------")


        #################### RETURN REWARD ####################

        # Always return a float value
        return float(reward)


reward_object = Reward(verbose=True)  # add parameter verbose=True to get noisy output for testing


def reward_function(params):
    return reward_object.reward_function(params)