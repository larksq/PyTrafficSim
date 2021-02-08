from graphics import *
from util import *
import random
import time
import math
import numpy as np
from vehicle_model import VehicleModels
from motion_planning import MotionPlanning
from policy import BasicPolicy
from controller import BasicController
import LaneChooser

from map import DefaultMap_4ways
from map import DefaultMap_4ways_WithLeftTurn
from map import DefaultMap_4ways_StopSigns

from dataloader.DataLoader import *


# max_steer = np.radians(30.0)  # [rad] max steering angle
# L = 2.9  # [m] Wheel base of vehicle
# dt = 0.1
# Lr = L / 2.0  # [m]
# Lf = L - Lr
# Cf = 1600.0 * 2.0  # N/rad
# Cr = 1700.0 * 2.0  # N/rad
# Iz = 2250.0  # kg/m2
# m = 1500.0  # kg


class Agent:
    def __init__(self,
                 # init location, angle, velocity
                 x=0.0, y=0.0, yaw=0.0, vx=0.01, vy=0, omega=0.0,
                 # Dynamic parameters, Cf and Cr: N/rad, Iz: kg/m2, m: kg
                 max_steer=np.radians(30.0), L=2.82, centerPer=0.45,
                 Cf=1600.0 * 2.0, Cr=1700.0 * 2.0, Iz=2250.0, m=1500.0,
                 # visualization dimension from Audi A4
                 length=4.726, width=1.842, Lf=0.45 * 82, goal_pts=(500, 500), goal_direction=0,
                 policy=None, spawn_position=(0, 0),
                 controller=None, cruise_speed=0.01, start_point=(0, 0)):

        self.x = x  # px
        self.y = y
        self.yaw = yaw
        self.vx = vx  # px/frame
        self.vy = vy
        self.omega = omega  # rate of steering
        # Aerodynamic and friction coefficients
        self.c_a = 1.36
        self.c_r1 = 0.01
        self.max_steer = max_steer  # [rad] max steering angle
        self.L = L  # px
        self.Lr = L - L * centerPer
        self.Lf = Lf
        self.Cf = Cf
        self.Cr = Cr
        self.Iz = Iz
        self.m = m
        self.length = length  # px
        self.width = width  # px
        self.a = 0
        self.policy = policy
        self.__goal = [goal_pts, goal_direction]  # normalize_angle(yaw-math.pi*0.5)]
        self.starting_pt = start_point
        self.direction_in_text = None  # you cannot get specific goal, but can get the direction in "L", "R", "F"
        self.spawn_position = spawn_position  # number of intersection, number of lane from the map
        self.trajectory = []  # [p, v, a] in meters and seconds
        self.path_polys = []
        self.agent_polys = []
        self.crashed = False
        self.cruise_speed = cruise_speed
        self.next_action = "normal"
        self.last_action_time = None
        self.controller = controller
        self.motion_planner = None
        self.points_before_next_stop_line = 0

    def gen_trajectory(self, scale, frame_rate):
        if len(self.trajectory) > 0:
            print("trajectory already inited")
            # if len(self.trajectory[0])>0:
            #     self.trajectory[0].pop(0)
            #     self.trajectory[1].pop(0)
            #     self.trajectory[2].pop(0)
        else:
            # init motion planner
            x, y = self.starting_pt
            current_p = (x / scale, y / scale)  # px->m
            current_v = self.vx / scale * frame_rate  # px/f->m/s
            current_a = self.a / scale * (frame_rate ** 2)
            current_theta = normalize_angle(self.yaw + 0.5 * math.pi)  # direction_angle
            c_state = [current_p, current_v, current_a, current_theta]
            t_state = [(self.__goal[0][0] / scale, self.__goal[0][1] / scale), current_v, current_a, self.__goal[1]]
            self.motion_planner = MotionPlanning.PolynomialTrajectaryGenerator(current_state=c_state,
                                                                               target_state=t_state,
                                                                               speed_limit=23)  # 23m/s=50mph
            self.motion_planner.fit_poly()
            self.trajectory = self.motion_planner.get_states(frame_rate)

    def update_trajectory(self, velocity):
        # use agent.v vm/s -> pop v/traj.sample_rate traj point
        # print("tset: ", int(velocity / self.motion_planner.sample_rate), velocity)
        idx = int(velocity / self.motion_planner.sample_rate)
        if idx > 0:
            for i in range(3):
                self.trajectory[i] = self.trajectory[i][int(velocity / self.motion_planner.sample_rate):]
        if self.points_before_next_stop_line > 0:
            self.points_before_next_stop_line -= idx

    def gen_trajectory_agent2pt(self, scale, frame_rate, t_state):
        current_p = (self.x / scale, self.y / scale)  # px->m
        current_v = self.vx / scale * frame_rate  # px/f->m/s
        current_a = self.a / scale * (frame_rate ** 2)
        current_theta = normalize_angle(self.yaw + 0.5 * math.pi)  # direction_angle
        c_state = [current_p, current_v, current_a, current_theta]
        motion_planner = MotionPlanning.PolynomialTrajectaryGenerator(current_state=c_state, target_state=t_state,
                                                                      speed_limit=23)  # 23m/s=50mph
        motion_planner.fit_poly()
        return motion_planner.get_states(frame_rate)

    def extend_trajectory(self, extend_in_meters, frame_rate, scale):
        # should be extended right after trajectory inited
        # t = extend_in_meters * scale  # frames
        extend_list_p = []
        extend_list_v = []
        extend_list_a = []
        total_points = int(extend_in_meters * self.motion_planner.sample_rate * frame_rate)
        self.points_before_next_stop_line = total_points
        delta_x = (self.starting_pt[0] - self.x) / total_points
        delta_y = (self.starting_pt[1] - self.y) / total_points
        vx = self.vx * math.cos(self.yaw) / scale * frame_rate
        vy = self.vx * math.sin(self.yaw) / scale * frame_rate
        for i in range(total_points):
            x = self.x + i * delta_x
            y = self.y + i * delta_y
            extend_list_p.append((x / scale, y / scale))
            extend_list_v.append((vx, vy))
            extend_list_a.append((0, 0))

        self.trajectory = [extend_list_p + self.trajectory[0],
                           extend_list_v + self.trajectory[1],
                           extend_list_a + self.trajectory[2]]

    def draw_traj(self, scale, win, clear_traj=False):
        skip_freq = 50 * self.motion_planner.sample_rate
        for i in range(0, len(self.path_polys)):
            self.path_polys[i].undraw()
        self.path_polys = []
        if not clear_traj:
            for i in range(0, len(self.trajectory[0])):
                if i % skip_freq == 0:
                    x, y = self.trajectory[0][i]
                    aPoint = Point(x * scale, y * scale)
                    aPoint.setFill("green")
                    aPoint.draw(win)
                    self.path_polys.append(aPoint)


class IntersectionSim:
    def __init__(self, win, map_cls=DefaultMap_4ways.Map, scale=15,
                 frame_rate=30.0, window_h=1000, window_w=1000, extend=17, trajectory=True, spawn_vertical_margin=5,
                 collision_speculate_skip_rate=15, speed_decrease_when_crowded=1.5, initial_speed_rate=1, cruising_speed_rate=1):
        self.map = map_cls()
        self.scale = scale
        self.win = win
        self.window_w = window_w
        self.window_h = window_h
        self.agents = []
        self.extend = extend
        self.frame_rate = frame_rate
        self.model = VehicleModels.DefaultKinematicModel(dt=1.0)
        self.simulate_dynamics = False
        self.trajectory = trajectory
        self.collision_detect = True
        self.finished_agent_id = []
        self.traffic_lights = self.init_traffic_lights_from_map()
        self.finished_agent = 0
        self.crash_time = 0
        self.spawn_vertical_margin = spawn_vertical_margin
        self.speed_decrease_when_crowded = speed_decrease_when_crowded  # divide 1.5 per 4 vehicles
        self.collision_speculate_skip_rate = collision_speculate_skip_rate  # the larger the faster and more likely to miss a cross collision
        self.initial_speed_rate = initial_speed_rate
        self.cruising_speed_rate = cruising_speed_rate

    def init_traffic_lights_from_map(self):
        schedules = self.map.traffic_lights_schedules
        positions = []
        directions = []
        if not len(schedules[0][1]["color"]) == len(self.map.roads):
            print("ERROR: traffic lights schedule number not fit to the road number")
            return 0
        for road in self.map.roads:
            x, y = road["pc"]
            x *= self.scale
            y *= self.scale
            x_new, y_new = tuple_recenter(point=(x, y),
                                          window_w=self.window_w,
                                          window_h=self.window_h)
            positions.append((x_new, y_new))
            directions.append(road["direction"])

        traffic_lights = TrafficLights(traffic_lights_schedules=schedules,
                                       positions=positions,
                                       directions=directions,
                                       scale=self.scale)
        return traffic_lights

    def draw_map(self):
        # assume 4-ways intersection
        line_extended = self.extend
        h = self.window_h
        w = self.window_w

        center_poly_points = []
        line_list_out = []
        line_list_in = []

        for road_dic in self.map.roads:
            pc = (road_dic["pc"][0] * self.scale, road_dic["pc"][1] * self.scale)
            direction = road_dic["direction"]
            length = road_dic["length"]
            out_no = road_dic["out_number"]
            in_no = road_dic["in_number"]

            rect_rotated_1 = tuple_recenter(
                    rotate(pc, (-in_no * length * self.scale + pc[0], pc[1]), direction, tuple=True), w, h)
            rect_rotated_2 = tuple_recenter(
                    rotate(pc, (out_no * length * self.scale + pc[0], pc[1]), direction, tuple=True), w, h)
            rect_rotated_3 = tuple_recenter(
                    rotate(pc, (out_no * length * self.scale + pc[0], pc[1] + line_extended * self.scale), direction,
                           tuple=True), w, h)
            rect_rotated_4 = tuple_recenter(
                    rotate(pc, (-in_no * length * self.scale + pc[0], pc[1] + line_extended * self.scale), direction,
                           tuple=True), w, h)

            a_polygon = Polygon(Point(rect_rotated_1[0], rect_rotated_1[1]),
                                Point(rect_rotated_2[0], rect_rotated_2[1]),
                                Point(rect_rotated_3[0], rect_rotated_3[1]),
                                Point(rect_rotated_4[0], rect_rotated_4[1]))

            a_polygon.setFill("black")
            a_polygon.draw(self.win)

            for i in range(0, out_no):
                rotated_fsp_1 = tuple_recenter(
                        rotate(pc, (i * length * self.scale + pc[0], pc[1]), direction, tuple=True), w, h)
                rotated_fsp_2 = tuple_recenter(
                        rotate(pc, ((i + 1) * length * self.scale + pc[0], pc[1]), direction, tuple=True), w, h)
                rotated_fsp_3 = tuple_recenter(
                        rotate(
                                origin=pc,
                                point=(i * length * self.scale + pc[0], pc[1] + line_extended * self.scale),
                                angle=direction,
                                tuple=True), w, h)
                line_list_out.append(Line(Point(rotated_fsp_1[0], rotated_fsp_1[1]),
                                          Point(rotated_fsp_2[0], rotated_fsp_2[1])))
                line_list_out.append(Line(Point(rotated_fsp_1[0], rotated_fsp_1[1]),
                                          Point(rotated_fsp_3[0], rotated_fsp_3[1])))

            for i in range(1, in_no):
                rotated_fsp_1 = tuple_recenter(
                        rotate(pc, (-i * length * self.scale + pc[0], pc[1]), direction, tuple=True), w, h)
                rotated_fsp_3 = tuple_recenter(
                        rotate(pc, (-i * length * self.scale + pc[0], pc[1] + line_extended * self.scale), direction,
                               tuple=True), w, h)
                line_list_in.append(
                        Line(Point(rotated_fsp_1[0], rotated_fsp_1[1]),
                             Point(rotated_fsp_3[0], rotated_fsp_3[1])))

            center_poly_points.append(rect_rotated_2)
            center_poly_points.append(rect_rotated_1)

        # draw center area
        c_polygon = Polygon(Point(center_poly_points[0][0], center_poly_points[0][1]),
                            Point(center_poly_points[1][0], center_poly_points[1][1]),
                            Point(center_poly_points[2][0], center_poly_points[2][1]),
                            Point(center_poly_points[3][0], center_poly_points[3][1]),
                            Point(center_poly_points[4][0], center_poly_points[4][1]),
                            Point(center_poly_points[5][0], center_poly_points[5][1]),
                            Point(center_poly_points[6][0], center_poly_points[6][1]),
                            Point(center_poly_points[7][0], center_poly_points[7][1]), )

        c_polygon.setFill("black")
        c_polygon.draw(self.win)

        for line in line_list_out:
            line.setFill("white")
            line.draw(self.win)

        for line in line_list_in:
            line.setFill("cyan")
            line.draw(self.win)

    def random_spawn_agent(self):
        speed_random_range = 0.1
        speed_random_bias = (random.random() % 1 - 0.5) / 0.5 * speed_random_range
        cruising_speed = 10 * self.scale / self.frame_rate * (1 + speed_random_bias)
        # start with low speed if crowded
        init_speed = 10 * self.initial_speed_rate * self.scale / self.frame_rate / max(
                (len(self.agents) // 4 * self.speed_decrease_when_crowded),
                1) * (1 + speed_random_bias)

        traffic_lights_status = self.traffic_lights.current_status
        current_traffic_dic = self.traffic_lights.traffic_lights_schedules[traffic_lights_status][1]
        total_road_numbers = len(self.map.roads)
        colors = current_traffic_dic["color"]
        rules = current_traffic_dic["rule"]
        green_light_roads = []
        green_light_directions = []
        for i, color in enumerate(colors):
            if len(self.traffic_lights.flashing) == len(colors) and self.traffic_lights.flashing[
                i] and color is not "yellow":
                return None
            if color in ["green", "green_left", "yellow"]:
                this_road_green_light_directions = []
                green_light_roads.append(i)
                directions = rules[i]
                if directions[0]:
                    this_road_green_light_directions.append("L")
                if directions[1]:
                    this_road_green_light_directions.append("F")
                if directions[2]:
                    this_road_green_light_directions.append("R")
                green_light_directions.append(this_road_green_light_directions)

        road_numbers = len(green_light_roads)
        selected_id = int(random.random() % 1 * road_numbers)
        intersection = green_light_roads[selected_id]
        road_dic = self.map.roads[intersection]
        # out_no = road_dic["out_number"]
        direction = road_dic["direction"]
        green_light_lanes = []
        for i, lane_rule in enumerate(road_dic["rules"]):
            for lane_direction in lane_rule["directions"]:
                if lane_direction in green_light_directions[selected_id]:
                    green_light_lanes.append(i)
                    break
        green_light_random_number = int(random.random() % 1 * len(green_light_lanes))
        spawn_lane_number = green_light_lanes[green_light_random_number]

        # check traffic law for the outbound lane
        rules = road_dic["rules"]
        specified_inbound = rules[spawn_lane_number]["outbounds"]
        if not len(specified_inbound):
            # did not specify which lane to go
            available_directions = rules[spawn_lane_number]["directions"]
            target_direction = available_directions[int(random.random() % 1 * len(available_directions))]
            if target_direction == "L":
                # this is a left turning lane
                goal_way = (intersection + 1) % total_road_numbers
            elif target_direction == "F":
                # this is a forward lane
                goal_way = (intersection + 2) % total_road_numbers
            elif target_direction == "R":
                # this is a right turning lane
                goal_way = (intersection - 1) % total_road_numbers
            else:
                print("ERROR: unknown direction for outbound lane: ", target_direction)
        else:
            print('# TODO: specified lanes to go')
            target_direction = "F"

        goal_road_dic = self.map.roads[goal_way]
        g_in_no = goal_road_dic["in_number"]
        # g_way_no = int(random.random() % 1 * g_in_no)  # get a total random lane to go
        g_way_no = LaneChooser.normal_lane_chooser(inbound_total_lanes=g_in_no,
                                                   outbound_lane_number=green_light_random_number,
                                                   outbound_total_lanes=len(green_light_lanes),
                                                   direction=target_direction)
        # g_way_no = LaneChooser.turning_random(inbound_total_lanes=g_in_no,
        #                                       outbound_lane_number=green_light_random_number,
        #                                       outbound_total_lanes=len(green_light_lanes),
        #                                       direction=target_direction)


        rotated_spawn_pt = self.map.get_point_from_map(intersection=intersection,
                                                       lane=spawn_lane_number,
                                                       scale=self.scale,
                                                       window_size=(self.window_w, self.window_h),
                                                       outbound=True, extend=self.extend)
        g_rotated_spawn_pt = self.map.get_point_from_map(intersection=goal_way,
                                                         lane=g_way_no,
                                                         scale=self.scale,
                                                         window_size=(self.window_w, self.window_h),
                                                         outbound=False)
        vehicle_size = 1.842 * self.scale, 4.726 * self.scale
        rotated_extended_pt = get_extended_point(rotated_spawn_pt, normalize_angle(direction + math.pi),
                                                 self.extend * self.scale)
        controller = BasicController.PIDController()
        policy = BasicPolicy.YieldOrStop(scale=self.scale, frame_rate=self.frame_rate, vehicle_size=vehicle_size,
                                         skip_rate=self.collision_speculate_skip_rate, alert_speed=cruising_speed*0.7)
        spawned_agent = Agent(x=rotated_extended_pt[0],
                              y=rotated_extended_pt[1],
                              yaw=normalize_angle(direction - 0.5 * math.pi),
                              vx=init_speed,
                              Lf=2.82 * 0.45 * self.scale,
                              width=vehicle_size[0],
                              length=vehicle_size[1],
                              goal_pts=g_rotated_spawn_pt,
                              goal_direction=normalize_angle(goal_road_dic["direction"] + math.pi),
                              spawn_position=(intersection, spawn_lane_number),  # px and px/f
                              cruise_speed=cruising_speed, controller=controller,
                              policy=policy, start_point=rotated_spawn_pt)
        spawned_agent.direction_in_text = target_direction

        # check position is occupied
        for agent in self.agents:
            if check_collision_for_two_agents(spawned_agent, agent, vertical_margin=self.spawn_vertical_margin):
                # print("spawned to an occupied lane")
                return

        # draw
        agent_w = spawned_agent.width * self.scale
        agent_l = spawned_agent.length * self.scale
        pt1, pt2, pt3, pt4 = generate_contour_pts(rotated_spawn_pt, agent_w, agent_l, direction)
        spawned_agent.agent_polys = Polygon(Point(pt1[0], pt1[1]),
                                            Point(pt2[0], pt2[1]),
                                            Point(pt3[0], pt3[1]),
                                            Point(pt4[0], pt4[1]))

        spawned_agent.agent_polys.setFill("green")
        spawned_agent.agent_polys.draw(self.win)

        self.agents.append(spawned_agent)
        return spawned_agent

    def speculate(self):
        # use vehicle policy to decide their next actions
        # px_per_frame_to_meter_per_second = self.frame_rate / self.scale
        meter_per_second_to_px_per_frame = self.scale / self.frame_rate
        mss_to_pff = self.scale / self.frame_rate / self.frame_rate
        for checking_agent in self.agents:
            if len(checking_agent.trajectory) > 1:
                turning_idle_speed = checking_agent.cruise_speed
                next_action = checking_agent.policy.get_next_action(checking_agent=checking_agent,
                                                                    all_agents=self.agents,
                                                                    idle_speed=turning_idle_speed)
                if next_action is not checking_agent.next_action:
                    if checking_agent.last_action_time is None or time.time() - checking_agent.last_action_time > checking_agent.policy.action_delay:
                        checking_agent.last_action_time = time.time()
                        checking_agent.next_action = next_action

                # dynamic
                if self.simulate_dynamics:
                    if next_action == "stop":
                        checking_agent.a = -15 * meter_per_second_to_px_per_frame
                    elif next_action == "yield":
                        checking_agent.a = checking_agent.policy.yield_a * meter_per_second_to_px_per_frame
                    elif next_action == "alert":
                        if checking_agent.vx >= checking_agent.policy.alert_speed * meter_per_second_to_px_per_frame:
                            checking_agent.a = checking_agent.policy.yield_a * meter_per_second_to_px_per_frame
                        else:
                            checking_agent.a = 0
                    elif next_action == "normal":
                        if checking_agent.vx >= checking_agent.cruise_speed:
                            checking_agent.a = 0
                        else:
                            checking_agent.a = checking_agent.policy.accelerate_rate * meter_per_second_to_px_per_frame
                # non-dynamic
                else:
                    checking_agent.a = 0
                    if checking_agent.next_action == "stop":
                        checking_agent.a = -15 * mss_to_pff
                        checking_agent.agent_polys.setFill("white")
                    elif checking_agent.next_action == "yield":
                        checking_agent.agent_polys.setFill("pink")
                        if checking_agent.policy.next_collision_pt is not None:
                            collision_pt_x, collision_pt_y = checking_agent.policy.next_collision_pt
                            collision_pt_x *= self.scale
                            collision_pt_y *= self.scale
                            distance = euclidean_distance(pt1=(checking_agent.x, checking_agent.y),
                                                          pt2=(collision_pt_x, collision_pt_y))
                            # if distance < checking_agent.policy.min_distance_to_slow * self.scale:
                            if checking_agent.policy.etc < checking_agent.policy.time_to_squeeze * self.frame_rate:
                                # Squeeze: I am blocking the road, accelerate to alert speed and leave
                                if checking_agent.vx < checking_agent.policy.alert_speed:
                                    checking_agent.a = checking_agent.policy.accelerate_rate * mss_to_pff * 0.8
                                    checking_agent.agent_polys.setFill("yellow")
                            elif distance < checking_agent.policy.distance_to_slow * self.scale:
                                # I am about to collide, time to slow down
                                if checking_agent.vx > 0:
                                    checking_agent.a = checking_agent.policy.yield_a * mss_to_pff
                            else:
                                if checking_agent.vx < checking_agent.policy.alert_speed:
                                    checking_agent.a = checking_agent.policy.accelerate_rate * mss_to_pff
                        else:
                            print("Warning: yield before detecting collision point")
                            # checking_agent.vx += checking_agent.policy.yield_a * mss_to_pff
                    elif checking_agent.next_action == "yield_squeeze":
                        if checking_agent.vx > 0:
                            checking_agent.agent_polys.setFill("pink")
                            checking_agent.a = checking_agent.policy.yield_a * mss_to_pff
                    elif checking_agent.next_action == "follow":
                        checking_agent.agent_polys.setFill("purple")
                        follow_agent = checking_agent.policy.follow_agent
                        if follow_agent is not None:
                            distance = euclidean_distance(pt1=(checking_agent.x, checking_agent.y),
                                                          pt2=(follow_agent.x, follow_agent.y))
                            if distance < checking_agent.policy.min_distance_to_slow * self.scale:
                                # probably already crashed into the other
                                print("Am I crashing into my follow agent?")
                                checking_agent.a = -10
                            elif distance < checking_agent.policy.distance_to_slow * self.scale * 1.2:
                                # I am about to collide, time to slow down
                                if checking_agent.vx > 0:
                                    checking_agent.a = checking_agent.policy.yield_a * mss_to_pff
                            else:
                                # I am too far from my follow agent, accelerate
                                if checking_agent.vx < checking_agent.policy.alert_speed:
                                    checking_agent.a = checking_agent.policy.accelerate_rate * mss_to_pff
                        else:
                            print("ERROR: no agent to follow!")

                    elif checking_agent.next_action == "alert":
                        checking_agent.agent_polys.setFill("cyan")
                        if checking_agent.vx >= checking_agent.policy.alert_speed * meter_per_second_to_px_per_frame:
                            if checking_agent.vx > 0:
                                checking_agent.a = checking_agent.policy.yield_a * mss_to_pff
                        else:
                            checking_agent.a = checking_agent.policy.accelerate_rate * mss_to_pff
                    elif checking_agent.next_action == "normal":
                        checking_agent.agent_polys.setFill("green")
                        if checking_agent.vx < checking_agent.cruise_speed:
                            checking_agent.a = checking_agent.policy.accelerate_rate * mss_to_pff

                    checking_agent.vx += checking_agent.a

    def check_collision(self):
        unchecked_agents = self.agents.copy()
        total_agents = len(unchecked_agents)
        if total_agents < 2:
            return
        # print("checking collision for ", total_agents, " agnets")
        checking_agent = unchecked_agents[0]
        for i in range(total_agents - 1):
            current_id = unchecked_agents.index(checking_agent)
            distances = []
            unchecked_agents.pop(current_id)
            # find the nearest agent
            for j in range(len(unchecked_agents)):
                pt1 = (checking_agent.x, checking_agent.y)
                pt2 = (unchecked_agents[j].x, unchecked_agents[j].y)
                dist = manhattan_distance(pt1, pt2)
                distances.append(dist)
            minimal_idx = distances.index(min(distances))
            target_agent = unchecked_agents[minimal_idx]
            is_collision = check_collision_for_two_agents(checking_agent, target_agent)
            if is_collision:
                id1 = self.agents.index(checking_agent)
                id2 = self.agents.index(target_agent)
                self.agents[id1].agent_polys.setFill("red")
                self.agents[id2].agent_polys.setFill("red")
                if not self.agents[id1].crashed or not self.agents[id2].crashed:
                    self.crash_time += 1
                self.agents[id1].crashed = True
                self.agents[id2].crashed = True
            checking_agent = target_agent

    def update(self):
        self.traffic_lights.update_traffic_lights(self.win)
        self.traffic_lights.check_flashing(self.win)
        if self.simulate_dynamics:
            # follow the dynamics
            for i, agent in enumerate(self.agents):
                agent.gen_trajectory(scale=self.scale, frame_rate=self.frame_rate)
                if self.trajectory:
                    if len(agent.trajectory[0]) > 1:
                        agent.draw_traj(scale=self.scale, win=self.win)
                    else:
                        agent.draw_traj(scale=self.scale, win=self.win, clear_traj=True)
                # self.model.update(agent=agent, throttle=-0.1, delta=0.1)
                throttle, delta = agent.controller.get_control_signal(agent=agent, scale=self.scale,
                                                                      frame_rate=self.frame_rate)
                throttle = throttle * self.scale / self.frame_rate / self.frame_rate
                delta = delta / self.frame_rate / self.frame_rate
                # NOTE: Current logic does not work
                self.model.update(agent=agent, throttle=0, delta=0)
                self.draw_agent_on_window(agent)
        else:
            # follow trajectory
            for i, agent in enumerate(self.agents):
                if len(agent.trajectory) <= 0:
                    # init a trajectory
                    agent.gen_trajectory(scale=self.scale, frame_rate=self.frame_rate)
                    agent.extend_trajectory(extend_in_meters=self.extend, frame_rate=self.frame_rate, scale=self.scale)
                else:
                    # update trajectory
                    agent.update_trajectory(agent.vx / self.scale * self.frame_rate)
                if len(agent.trajectory[0]) > 1:
                    # follow trajectory
                    if self.trajectory:
                        agent.draw_traj(scale=self.scale, win=self.win)
                    self.update_agent_without_model(agent, self.scale, self.frame_rate)
                    self.draw_agent_on_window(agent)
                else:
                    # end the agent at its destination
                    if self.trajectory:
                        agent.draw_traj(scale=self.scale, win=self.win, clear_traj=True)
                    self.draw_agent_on_window(agent, clear_agent=True)
                    self.finished_agent_id.append(i)
                    self.finished_agent += 1

    def loop(self, agents_per_second=5, frame_rate=30, running_time=10):
        # running_time = running_time # seconds
        self.draw_map()
        self.random_spawn_agent()
        self.frame_rate = frame_rate
        last_time = time.time()
        last_spawn = time.time()
        init_time = last_time
        counter = 0
        spawn_freq = agents_per_second  # n agents per second
        # for i in range(0, 300):
        self.traffic_lights.init_traffic_lights(win=self.win)

        while counter < running_time * self.frame_rate:
            if len(self.agents) > 50:
                print("Quit simulation: too many agents")
                return
            current_time = time.time()
            if current_time - last_spawn > 1.0 / spawn_freq:
                self.random_spawn_agent()
                last_spawn = current_time
            if current_time - last_time > 1.0 / self.frame_rate:
                self.speculate()
                self.update()
                update(30)
                last_time = current_time
                counter += 1
                self.check_collision()
                self.clear_finished_agents()
            else:
                time.sleep(1.0 / self.frame_rate / 40.0)
        print("LOOP END. \nSupposed Running time: " + str(running_time) +
              "\nActual Running time: " + str(time.time() - init_time) + "s" +
              "\nFPS: " + str(int(counter / (time.time() - init_time))) + "/" + str(int(self.frame_rate)))
        system_log = "Simulation Finished\n\nCrashed: {}\nTotal traffic: {}\nSimulation time: {}".format(
                self.crash_time, self.finished_agent, str(running_time))
        message = Text(Point(self.window_w / 2, self.window_h / 2), system_log + "\n\nClick anywhere to close")
        message.setTextColor("pink")
        message.setSize(22)
        message.draw(graphwin=self.win)

    @staticmethod
    def update_agent_without_model(agent, scale, frame_rate):
        x, y = agent.trajectory[0][1]
        v_x, v_y = agent.trajectory[1][1]
        # a_x, a_y = agent.trajectory[0][2]
        agent.x = x * scale
        agent.y = y * scale
        # agent.vx = math.sqrt((v_x*scale/frame_rate)**2+(v_y*scale/frame_rate)**2)
        agent.yaw = normalize_angle(math.atan2(v_y * frame_rate, v_x * frame_rate))

    def draw_agent_on_window(self, agent, clear_agent=False):
        poly = agent.agent_polys
        poly.undraw()
        if not clear_agent:
            pt1, pt2, pt3, pt4 = generate_contour_pts((agent.x, agent.y), agent.width, agent.length,
                                                      normalize_angle(agent.yaw + 0.5 * math.pi))
            poly.points = [Point(pt1[0], pt1[1]), Point(pt2[0], pt2[1]), Point(pt3[0], pt3[1]), Point(pt4[0], pt4[1])]
            poly.draw(self.win)

    def clear_finished_agents(self):
        if len(self.finished_agent_id) > 1:
            print("early finished agents: ", self.finished_agent_id, len(self.agents))
        if len(self.finished_agent_id) > 0:
            # multiple ids can exits, and they are ascending
            counter = 0
            for i in self.finished_agent_id:
                self.agents.pop(i - counter)
                counter += 1
            self.finished_agent_id = []


class TrafficLights:
    def __init__(self, traffic_lights_schedules, positions, directions, scale=1):
        # all schedules, positions are stored in a list for each road on the map
        # number of traffic lights and the roads in the map should be equal and sorted
        self.traffic_lights_schedules = traffic_lights_schedules
        self.positions = positions
        self.directions = directions
        self.init_drawn = False
        # for each light, there is a rect and an oval in a list
        self.lights_shapes = []
        self.current_status = 0
        self.size = scale * 2
        self.scale = scale
        self.time_to_next_state = 0
        self.current_rules = None
        self.flashing = []
        self.poly_drawn = []
        self.left_turn = []  # [True, False, True, False] True for current left turning lights

    def init_traffic_lights(self, win):
        self.init_drawn = True
        # init polygons to shape list
        current_schedule = self.traffic_lights_schedules[self.current_status]
        self.time_to_next_state = current_schedule[0] + time.time()
        self.current_rules = current_schedule[1]["rule"]
        self.flashing = current_schedule[1]["flashing"]
        for i, position in enumerate(self.positions):
            # for each traffic light
            pc_x, pc_y = position
            direction = normalize_angle(self.directions[i] + math.pi / 2)
            an_oval = Oval(Point(pc_x - self.size / 2, pc_y - self.size / 2),
                           Point(pc_x + self.size / 2, pc_y + self.size / 2))
            # facing to the left as 0
            left_upper = rotate(origin=(pc_x, pc_y),
                                point=(pc_x, pc_y - self.size / 2),
                                angle=direction,
                                tuple=True)
            right_upper = rotate(origin=(pc_x, pc_y),
                                 point=(pc_x + self.size / 2, pc_y - self.size / 2),
                                 angle=direction,
                                 tuple=True)
            right_lower = rotate(origin=(pc_x, pc_y),
                                 point=(pc_x + self.size / 2, pc_y + self.size / 2),
                                 angle=direction,
                                 tuple=True)
            left_lower = rotate(origin=(pc_x, pc_y),
                                point=(pc_x, pc_y + self.size / 2),
                                angle=direction,
                                tuple=True)
            a_polygon = Polygon(Point(left_upper[0], left_upper[1]),
                                Point(right_upper[0], right_upper[1]),
                                Point(right_lower[0], right_lower[1]),
                                Point(left_lower[0], left_lower[1]))
            left_upper_block = rotate(origin=(pc_x, pc_y),
                                      point=(pc_x - self.size / 2, pc_y),
                                      angle=direction,
                                      tuple=True)
            right_upper_block = position
            right_lower_block = left_lower
            left_lower_block = rotate(origin=(pc_x, pc_y),
                                      point=(pc_x - self.size / 2, pc_y + self.size / 2),
                                      angle=direction,
                                      tuple=True)
            a_right_block = Polygon(Point(left_upper_block[0], left_upper_block[1]),
                                    Point(right_upper_block[0], right_upper_block[1]),
                                    Point(right_lower_block[0], right_lower_block[1]),
                                    Point(left_lower_block[0], left_lower_block[1]))
            a_right_block.setFill("black")
            detail_dic = current_schedule[1]

            if detail_dic["color"][i] == "red":
                an_oval.setFill("red")
            elif detail_dic["color"][i] in ["green", "green_left"]:
                an_oval.setFill("green")
            elif detail_dic["color"][i] == "yellow":
                an_oval.setFill("yellow")
            else:
                print("ERROR: unknown traffic light color: ", detail_dic["color"][i])

            a_polygon.setFill(color_rgb(130, 130, 130))
            an_oval.draw(win)
            a_polygon.draw(win)
            if detail_dic["color"][i] == "green_left":
                a_right_block.draw(win)
                self.left_turn.append(True)
            else:
                self.left_turn.append(False)
            poly_list = [an_oval, a_polygon, a_right_block]
            self.lights_shapes.append(poly_list)
            self.poly_drawn.append(True)

    def update_traffic_lights(self, win):
        if self.init_drawn and time.time() > self.time_to_next_state:
            # change status
            self.current_status = (self.current_status + 1) % len(self.traffic_lights_schedules)
            current_schedule = self.traffic_lights_schedules[self.current_status]
            self.current_rules = current_schedule[1]["rule"]
            self.flashing = current_schedule[1]["flashing"]
            detail_dic = current_schedule[1]
            self.time_to_next_state = current_schedule[0] + time.time()
            for i, poly_group in enumerate(self.lights_shapes):
                an_oval = poly_group[0]
                for shape in poly_group:
                    shape.undraw()
                if detail_dic["color"][i] == "red":
                    an_oval.setFill("red")
                elif detail_dic["color"][i] in ["green", "green_left"]:
                    an_oval.setFill("green")
                elif detail_dic["color"][i] == "yellow":
                    an_oval.setFill("yellow")
                else:
                    print("ERROR: unknown traffic light color: ", detail_dic["color"][i])

                if detail_dic["color"][i] == "green_left":
                    self.left_turn[i] = True
                    for shape in poly_group:
                        shape.draw(win)
                else:
                    self.left_turn[i] = False
                    poly_group[0].draw(win)
                    poly_group[1].draw(win)
                self.poly_drawn[i] = True

    def check_flashing(self, win):
        if self.init_drawn:
            for i, flashing in enumerate(self.flashing):
                if flashing:
                    drawn = self.poly_drawn[i]
                    if int(time.time()) % 2 and drawn:
                        self.lights_shapes[i][0].undraw()
                        self.poly_drawn[i] = False
                    elif not int(time.time()) % 2:
                        if not drawn:
                            # note draw an drawn object will cause error
                            self.lights_shapes[i][0].draw(win)
                            self.lights_shapes[i][1].undraw()
                            self.lights_shapes[i][1].draw(win)
                            if self.left_turn[i]:
                                self.lights_shapes[i][2].undraw()
                                self.lights_shapes[i][2].draw(win)
                            else:
                                self.lights_shapes[i][2].undraw()
                            self.poly_drawn[i] = True


class SimWithLoadedData(IntersectionSim):
    nuScenes_frame_rate = 2

    def __init__(self, win, scale=15,
                 window_h=1000, window_w=1000, trajectory=True,
                 collision_speculate_skip_rate=15):
        self.scale = scale
        self.win = win
        self.window_w = window_w
        self.window_h = window_h
        self.agents = []
        self.frame_rate = self.nuScenes_frame_rate
        self.model = VehicleModels.DefaultKinematicModel(dt=1.0)
        self.simulate_dynamics = False
        self.trajectory = trajectory
        self.collision_detect = True
        # self.traffic_lights = self.init_traffic_lights_from_map()
        self.collision_speculate_skip_rate = collision_speculate_skip_rate  # the larger the faster and more likely to miss a cross collision

        self.map_polys = []
        self.drawn_lane_tokens = []
        self.nusc_map = NuScenesMap(dataroot='data/sets/nuscenes', map_name='singapore-onenorth')
        self.loader = NuScenesLoader(scale=scale,
                                     frame_rate=self.nuScenes_frame_rate, window_h=1000, window_w=1000)

    def loop_with_loaded_data(self, frame_rate=2):
        # load agents from DataLoader
        frame_list = self.loader.load_a_scene(scene_num=0)

        last_time = time.time()

        bgMap = Image(Point(self.window_w / 2,self.window_h / 2), "images/NuScene_map01_rescaled.png")
        bgMap.draw(self.win)

        self.draw_predicted_agent_traj((393.357, 1149.173, 0.419), (385.638, 1131.14, 0.419), 13, 15, 0.1)
        self.draw_predicted_agent_traj((385.638, 1131.14, 0.419), (346.001, 1120.849, 1.868), 15, 15, 0.1 / 1.2)

        while len(frame_list) > 0:
            current_time = time.time()
            if current_time - last_time > 1.0 / self.nuScenes_frame_rate:
                # draw an agent on current frame
                # pop the first frame
                self.draw_agent_on_window_by_frame(frame_list)
                frame_list.pop(0)
                update(self.nuScenes_frame_rate)
                last_time = current_time
                # self.check_collision()
            else:
                time.sleep(1.0 / self.nuScenes_frame_rate / 40.0)

        print("finished")

    def draw_predicted_agent_traj(self, starting_pt, ending_pt, staring_v, ending_v, rate):
        x, y, yaw = starting_pt
        current_p = self.recenter((x, y), self.loader.offsets)
        current_v = staring_v
        current_a = 0
        current_theta = yaw#normalize_angle(yaw - 0.5 * math.pi)  # direction_angle
        ending_x, ending_y = self.recenter((ending_pt[0], ending_pt[1]), self.loader.offsets)
        c_state = [current_p, current_v, current_a, current_theta]
        t_state = [(ending_x, ending_y), ending_v, current_a, ending_pt[2]] # normalize_angle(ending_pt[2] - 0.5 * math.pi)]
        motion_planner = MotionPlanning.PolynomialTrajectaryGenerator(current_state=c_state,
                                                                      target_state=t_state,
                                                                      speed_limit=23)  # 23m/s=50mph
        motion_planner.fit_poly()
        trajectory = motion_planner.get_states(rate)
        for pt in trajectory[0]:
            x, y = pt
            aCircle = Circle(Point(x ,y), 1)
            aCircle.setFill("yellow")
            aCircle.draw(self.win)


    def init_map_drawing(self, frame_list):
        # only draw the map based on the 0 frame
        for agent_parameters in frame_list[0]:
            x, y, yaw, width, length, v, a = agent_parameters
            self.draw_temp_poses_and_lane(x=x, y=y, yaw=yaw)

    def draw_temp_poses_and_lane(self, x, y, yaw):
        closest_lane = self.nusc_map.get_closest_lane(x, y, radius=2)
        if closest_lane not in self.drawn_lane_tokens:
            lane_record = self.nusc_map.get_arcline_path(closest_lane)
            poses = arcline_path_utils.discretize_lane(lane_record, resolution_meters=1)
            for pose in poses:
                recentered_pose = self.recenter((pose[0], pose[1]), self.loader.offsets)
                pt1, pt2, pt3, pt4 = generate_contour_pts((recentered_pose[0], recentered_pose[1]),
                                                          10 * self.scale, 3.5 * self.scale, -pose[2])  # length, width
                aPoly = Polygon(Point(pt1[0], pt1[1]),
                                Point(pt2[0], pt2[1]),
                                Point(pt3[0], pt3[1]),
                                Point(pt4[0], pt4[1]))
                aPoly.setFill("black")
                aPoly.draw(self.win)
                self.map_polys.append(aPoly)
            self.drawn_lane_tokens.append(closest_lane)

    def draw_agent_on_window_by_frame(self, frame_list):
        for agent in self.agents:
            poly = agent.agent_polys
            poly.undraw()
        self.agents = []
        # self.init_map_drawing(frame_list)
        for agent_parameters in frame_list[0]:
            # for each agent
            x, y, yaw, width, length, v, a = agent_parameters
            offsets = self.loader.offsets
            recentered_xy = self.recenter((x, y), offsets)
            new_agent = Agent(x=recentered_xy[0],
                              y=recentered_xy[1], yaw=-yaw,
                              vx=v, length=length * self.scale, width=width * self.scale)
            self.agents.append(new_agent)
            # draw
            agent_w = new_agent.width
            agent_l = new_agent.length
            pt1, pt2, pt3, pt4 = generate_contour_pts((new_agent.x, new_agent.y), agent_w, agent_l, new_agent.yaw)
            new_agent.agent_polys = Polygon(Point(pt1[0], pt1[1]),
                                            Point(pt2[0], pt2[1]),
                                            Point(pt3[0], pt3[1]),
                                            Point(pt4[0], pt4[1]))
            new_agent.agent_polys.setFill("green")
            new_agent.agent_polys.draw(self.win)

    def recenter(self, pt, offsets):
        x, y = pt
        return - (x + offsets[0]) * self.scale + self.window_w / 2, (y + offsets[1]) * self.scale + self.window_h / 2


class DynamicTest:
    def __init__(self, win, frame_rate=30.0, window_h=1000, window_w=1000, lines_num=10):
        self.win = win
        self.model = VehicleModels.DefaultDynamicModel(dt=1.0)
        self.agentEgo = Agent()
        self.i = 1
        self.window_w = window_w
        self.window_h = window_h
        self.lines_num = lines_num
        self.lineList_h = []
        self.lineList_v = []
        self.offset_x = 0
        self.offset_y = 0
        self.rotation = 0

        for i in range(0, lines_num * 3):
            line_x = 1 + i * float(window_w) / lines_num
            line_y = 1 + i * float(window_h) / lines_num
            aline = Line(Point(-window_w + line_x, -window_h), Point(-window_w + line_x, window_h * 2))
            # aline.p1 = Point(line_x, 100)
            aline.draw(win)
            bline = Line(Point(-window_w, -window_h + line_y), Point(window_w * 2, -window_h + line_y))
            # bline = Line(Point(0, line_y), Point(window_w, line_y))
            bline.draw(win)
            self.lineList_v.append(aline)
            self.lineList_h.append(bline)

    def update(self, throttle=5, delta=0.05):

        # key = self.win.checkKey()
        # print("check key: ", key)

        self.model.update(agent=self.agentEgo, throttle=throttle, delta=delta)
        offset_x = - self.agentEgo.x * 10 % 100
        offset_y = - self.agentEgo.y * 10 % 100

        yaw = -self.agentEgo.yaw
        # yaw = 0
        # lines_num = 10 # draw grids, 1line/100px, 1m/10px

        for i, line_v in enumerate(self.lineList_v):
            line_x = offset_x + i * float(self.window_w) / self.lines_num
            pt_1 = (-self.window_w + line_x, -self.window_h)
            pt_2 = (-self.window_w + line_x, self.window_h * 2)
            pt_1_r = rotate((0, 0), pt_1, yaw)
            pt_2_r = rotate((0, 0), pt_2, yaw)

            line_v.undraw()
            line_v.p1 = Point(pt_1_r[0], pt_1_r[1])
            line_v.p2 = Point(pt_2_r[0], pt_2_r[1])
            line_v.draw(self.win)
            # line.move(offset_x, 1)

        for i, line_h in enumerate(self.lineList_h):
            line_y = offset_y + i * float(self.window_h) / self.lines_num
            pt_1 = (-self.window_w, -self.window_h + line_y)
            pt_2 = (self.window_w * 2, -self.window_h + line_y)
            pt_1_r = rotate((0, 0), pt_1, yaw)
            pt_2_r = rotate((0, 0), pt_2, yaw)

            line_h.undraw()
            line_h.p1 = Point(pt_1_r[0], pt_1_r[1])
            line_h.p2 = Point(pt_2_r[0], pt_2_r[1])
            line_h.draw(self.win)

            # update only once in the end

    def runLoop(self, fram_rate=30):
        print("run loop")

        self.update()

        w = self.agentEgo.width
        l = self.agentEgo.length
        carRec = Rectangle(Point(self.window_w / 2 - l * 10 / 2, self.window_h / 2 - w * 10 / 2),
                           Point(self.window_w / 2 + l * 10 / 2, self.window_h / 2 + w * 10 / 2))
        carRec.setFill("red")
        carRec.draw(self.win)

        for i in range(0, 300):
            self.update()
            carRec.undraw()
            carRec.draw(self.win)
            update(30)

        print("loop end")
