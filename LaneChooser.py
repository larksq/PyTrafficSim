import random
def normal_lane_chooser(inbound_total_lanes, outbound_lane_number, outbound_total_lanes, direction):
    # if forward, 0 for 0, 1 for 1
    if direction == "F":
        return int(inbound_total_lanes * outbound_lane_number / outbound_total_lanes)
    # if turning left choose the most left one to go
    elif direction == "L":
        return 0
    # if turning right choose the most right one to go
    elif direction == "R":
        return inbound_total_lanes - 1
    else:
        print("ERROR-LaneChooser: Unknown direction ", direction)

def turning_random(inbound_total_lanes, outbound_lane_number, outbound_total_lanes, direction,
                   left_random=True, right_random=True):
    random_result = int(random.random() % 1 * inbound_total_lanes)
    # if forward, 0 for 0, 1 for 1
    if direction == "F":
        return int(inbound_total_lanes * outbound_lane_number / outbound_total_lanes)
    # if turning left random inbound
    elif direction == "L":
        if left_random:
            return random_result
        else:
            return 0
    # if turning right random inbound
    elif direction == "R":
        if right_random:
            return random_result
        else:
            return inbound_total_lanes - 1
    else:
        print("ERROR-LaneChooser: Unknown direction ", direction)