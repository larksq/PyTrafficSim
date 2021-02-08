from util import *

# lane rules counting from the center towards the right (right side driving rules)
# empty outbounds -> default (Dynamic)
lanes_rule_3 = [{
    "directions": ["L"],
    "outbounds": [],
}, {
    "directions": ["F"],
    "outbounds": [],
}, {
    "directions": ["R", "F"],
    "outbounds": [],
}]
lanes_rule_4 = [{
    "directions": ["L"],
    "outbounds": [],
}, {
    "directions": ["F"],
    "outbounds": [],
}, {
    "directions": ["F"],
    "outbounds": [],
}, {
    "directions": ["R", "F"],
    "outbounds": [],
}]


class Map:
    def __init__(self):
        # from the Skinker-Forest intersection
        self.tl_yellow = [True, True, True]  # left, forward, right
        self.roads = [{
            "out_number": 3,
            "in_number": 3,
            "pc": (2.68, -18.35),
            "length": 3.09,
            "direction": -3.01942,
            "rules": lanes_rule_3
        }, {
            "out_number": 3,
            "in_number": 2,
            "pc": (15.29, 0.48),
            "length": 3.52,
            "direction": -1.48353,
            "rules": lanes_rule_3
        }, {
            "out_number": 4,
            "in_number": 2,
            "pc": (-4.12, 16.19),
            "length": 3.09,
            "direction": 0.122173,
            "rules": lanes_rule_4
        }, {
            "out_number": 4,
            "in_number": 2,
            "pc": (-16.16, -0.56),
            "length": 3.09,
            "direction": 1.65647,
            "rules": lanes_rule_4
        }]
        self.traffic_lights_schedules = [
            [
                99999999, {"color": ["yellow", "yellow", "yellow", "yellow"], "flashing": [True, True, True, True],
                     "rule": [self.tl_yellow, self.tl_yellow, self.tl_yellow, self.tl_yellow]},
            ]]

    def get_point_from_map(self, intersection, lane, scale, window_size, outbound=True, extend=0):
        window_w, window_h = window_size
        road_dic = self.roads[intersection]
        pc = (road_dic["pc"][0]*scale, (road_dic["pc"][1])*scale)
        length = road_dic["length"]
        direction = road_dic["direction"]
        if outbound:
            rotated_spawn_pt = tuple_recenter(rotate(pc, (pc[0]+length*(0.5+lane)*scale, pc[1]),
                                                     direction, tuple=True), window_w, window_h)
        else:
            # inbound
            rotated_spawn_pt = tuple_recenter(rotate(pc, (pc[0]+length*(-0.5-lane)*scale, pc[1]),
                                                     direction, tuple=True), window_w, window_h)
        return rotated_spawn_pt



