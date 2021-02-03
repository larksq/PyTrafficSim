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
        self.tl_red = [False, False, False]  # left, forward, right
        self.tl_green = [False, True, True]  # left, forward, right
        self.tl_green_left = [True, False, False]  # left, forward, right
        # from the Skinker-Forest intersection
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
                20, {"color": ["red", "green", "red", "green"], "flashing": [False, False, False, False],
                     "rule": [self.tl_red, self.tl_green, self.tl_red, self.tl_green]},
            ],
            [
                4, {"color": ["red", "green", "red", "green"], "flashing": [False, True, False, True],
                    "rule": [self.tl_red, self.tl_green, self.tl_red, self.tl_green]},
            ],
            [
                15, {"color": ["red", "green_left", "red", "green_left"], "flashing": [False, False, False, False],
                    "rule": [self.tl_red, self.tl_green_left, self.tl_red, self.tl_green_left]},
            ],
            [
                4, {"color": ["red", "green_left", "red", "green_left"], "flashing": [False, True, False, True],
                    "rule": [self.tl_red, self.tl_green_left, self.tl_red, self.tl_green_left]},
            ],
            [
                20, {"color": ["green", "red", "green", "red"], "flashing": [False, False, False, False],
                     "rule": [self.tl_green, self.tl_red, self.tl_green, self.tl_red]},
            ],
            [
                4, {"color": ["green", "red", "green", "red"], "flashing": [True, False, True, False],
                    "rule": [self.tl_green, self.tl_red, self.tl_green, self.tl_red]},
            ],
            [
                15, {"color": ["green_left","red", "green_left", "red"], "flashing": [False, False, False, False],
                    "rule": [self.tl_green_left, self.tl_red, self.tl_green_left, self.tl_red]},
            ],
            [
                4, {"color": ["green_left","red", "green_left", "red"], "flashing": [True, False, True, False],
                    "rule": [self.tl_green_left, self.tl_red, self.tl_green_left, self.tl_red]},
            ],]

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



