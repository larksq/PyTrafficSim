import math
import numpy as np


def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].
    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle


def rotate(origin, point, angle, tuple=False):
    """
    Rotate a point clockwise by a given angle around a given origin.
    The angle should be given in radians.
    """
    ox, oy = origin
    px, py = point

    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    if tuple:
        return (qx, qy)
    else:
        return qx, qy


def tuple_recenter(point, window_w, window_h, tuple=True):
    x, y = point
    if tuple:
        return (x+window_w/2, y+window_h/2)
    else:
        return x+window_w/2, y+window_h/2


def generate_contour_pts(center_pt, w, l, direction):
    pt1 = rotate(center_pt, (center_pt[0]-w/2, center_pt[1]-l/2), direction, tuple=True)
    pt2 = rotate(center_pt, (center_pt[0]+w/2, center_pt[1]-l/2), direction, tuple=True)
    pt3 = rotate(center_pt, (center_pt[0]+w/2, center_pt[1]+l/2), direction, tuple=True)
    pt4 = rotate(center_pt, (center_pt[0]-w/2, center_pt[1]+l/2), direction, tuple=True)
    return pt1, pt2, pt3, pt4


def euclidean_distance(pt1, pt2):
    x_1, y_1 = pt1
    x_2, y_2 = pt2
    return math.sqrt((x_1-x_2)**2+(y_1-y_2)**2)


def manhattan_distance(pt1, pt2):
    x_1, y_1 = pt1
    x_2, y_2 = pt2
    return abs(x_1-x_2)+abs(y_1-y_2)


def get_angle_of_a_line(pt1, pt2):
    # angle from horizon to the right, counter-clockwise,
    x1, y1 = pt1
    x2, y2 = pt2
    angle = math.atan2(y2 - y1, x2 - x1)
    return angle


def is_point_in_box(point_tuple, box_two_points):
    x, y = point_tuple
    box_1, box_2 = box_two_points
    upper_left_x, upper_left_y = box_1
    lower_right_x, lower_right_y = box_2
    if (lower_right_x - x) * (upper_left_x - x) <= 0:
        if (lower_right_y - y) * (upper_left_y - y) <= 0:
            return True
    return False


def is_point_in_box_with_angel(point_tuple, box_four_points):
    angles = []
    sum_result = 0
    for pt in box_four_points:
        angles.append(get_angle_of_a_line(point_tuple, pt))
    angles.append(angles[0])
    for i in range(len(angles)-1):
        sum_result += abs(normalize_angle(angles[i+1] - angles[i]))
    if sum_result >= math.pi*1.99:
        return True
    else:
        return False


def check_collision_for_two_agents(checking_agent, target_agent, vertical_margin=1):
    collision_box_c = [(checking_agent.x - checking_agent.width/2,
                        checking_agent.y - checking_agent.length/2 * vertical_margin),
                       (checking_agent.x - checking_agent.width/2,
                        checking_agent.y + checking_agent.length/2 * vertical_margin),
                       (checking_agent.x + checking_agent.width/2,
                        checking_agent.y + checking_agent.length/2 * vertical_margin),
                       (checking_agent.x + checking_agent.width/2,
                        checking_agent.y - checking_agent.length/2 * vertical_margin)]
    rotated_checking_box_c = []
    for pt in collision_box_c:
        rotated_checking_box_c.append(rotate(origin=(checking_agent.x, checking_agent.y),
                                             point=pt,
                                             angle=normalize_angle(checking_agent.yaw + math.pi / 2),
                                             tuple=True))

    collision_box_t = [(target_agent.x - target_agent.width/2,
                        target_agent.y - target_agent.length/2 * vertical_margin),
                       (target_agent.x - target_agent.width/2,
                        target_agent.y + target_agent.length/2 * vertical_margin),
                       (target_agent.x + target_agent.width/2,
                        target_agent.y + target_agent.length/2 * vertical_margin),
                       (target_agent.x + target_agent.width/2,
                        target_agent.y - target_agent.length/2 * vertical_margin)]
    rotated_checking_box_t = []
    for pt in collision_box_t:
        rotated_checking_box_t.append(rotate(origin=(target_agent.x, target_agent.y),
                                             point=pt,
                                             angle=normalize_angle(target_agent.yaw + math.pi / 2),
                                             tuple=True))
    return check_collision_for_two_center_points(rotated_checking_box_c, rotated_checking_box_t)


def check_collision_for_point_in_path(pt1, size1, yaw1, pt2, size2, yaw2, vertical_margin=1):
    x1, y1 = pt1
    x2, y2 = pt2
    width1, length1 = size1
    width2, length2 = size2
    collision_box_c = [(x1 - width1/2,
                        y1 - length1/2 * vertical_margin),
                       (x1 - width1/2,
                        y1 + length1/2 * vertical_margin),
                       (x1 + width1/2,
                        y1 + length1/2 * vertical_margin),
                       (x1 + width1/2,
                        y1 - length1/2 * vertical_margin)]
    rotated_checking_box_c = []
    for pt in collision_box_c:
        rotated_checking_box_c.append(rotate(origin=(x1, y1),
                                             point=pt,
                                             angle=normalize_angle(yaw1 + math.pi / 2),
                                             tuple=True))

    collision_box_t = [(x2 - width2/2,
                        y2 - length2/2 * vertical_margin),
                       (x2 - width2/2,
                        y2 + length2/2 * vertical_margin),
                       (x2 + width2/2,
                        y2 + length2/2 * vertical_margin),
                       (x2 + width2/2,
                        y2 - length2/2 * vertical_margin)]
    rotated_checking_box_t = []
    for pt in collision_box_t:
        rotated_checking_box_t.append(rotate(origin=(x2, y2),
                                             point=pt,
                                             angle=normalize_angle(yaw2 + math.pi / 2),
                                             tuple=True))
    return check_collision_for_two_center_points(rotated_checking_box_c, rotated_checking_box_t)


def check_collision_for_two_center_points(rotated_checking_box_c, rotated_checking_box_t):
    # check each point in/out of the box
    collision = False
    for pt in rotated_checking_box_t:
        if is_point_in_box_with_angel(pt, rotated_checking_box_c):
            collision = True
            break
    return collision


def get_possible_destinations(agent, direction, map, scale, window_size):
    destinations = []
    out_intersection, out_lane = agent.spawn_position
    total_roads_number = len(map.roads)
    if direction == "L":
        target_intersection = (out_intersection - 1) % total_roads_number
    elif direction == "R":
        target_intersection = (out_intersection + 1) % total_roads_number
    elif direction == "F":
        target_intersection = (out_intersection + 2) % total_roads_number
    else:
        print("get_possible_destinations - Error & Exiting: unknown direction ", direction)
    total_lanes_target = map.roads[target_intersection]["in_number"]
    target_direction = normalize_angle(map.roads[target_intersection]["direction"] + math.pi)
    for i in range(total_lanes_target):
        target_pt = map.get_point_from_map(intersection=target_intersection, lane=i, scale=scale, window_size=window_size)
        destinations.append([target_pt, target_direction])
    return destinations


def destinations_to_paths_batch(agent, destinations, scale, frame_rate, target_v, target_a=0):
    paths = []
    for destination in destinations:
        t_state = [(destination[0][0]/scale, destination[0][1]/scale), target_v, target_a, destination[1]]
        paths.append(agent.gen_trajectory_agent2pt(scale=scale, frame_rate=frame_rate, t_state=t_state))
    return paths


def get_extended_point(starting_point, direction, extend):
    return starting_point[0] + math.sin(direction) * extend, starting_point[1] - math.cos(direction) * extend
