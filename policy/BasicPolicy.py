from util import *


class YieldOrStop:
    def __init__(self, scale=1, frame_rate=30, unscaled_emergency_dist=10, alert_speed=7, yield_a=-10,
                 accelerate_rate=5,
                 vehicle_size=(1.842, 4.726), skip_rate=15):
        # all unit not scaled as meters, meters/second ..
        self.scale = scale
        self.emergency_dist = unscaled_emergency_dist * scale
        self.frame_rate = frame_rate
        self.alert_speed = alert_speed
        self.yield_a = yield_a
        self.accelerate_rate = accelerate_rate
        self.vehicle_size = vehicle_size  # width, length
        self.action_delay = 0.8  # driving reaction time in seconds
        self.distance_to_slow = 30  # in meters, also used for actions before stop line
        self.distance_to_stopline = 8
        self.min_distance_to_slow = 3  # in meters
        self.follow_agent = None
        # temporary public parameters, also used by action response
        self.next_collision_pt = None  # follow collision do not update collision point
        self.etc = None  # estimated time of collision in frames
        self.max_etc_for_follow_collision = None
        self.skip_rate = skip_rate
        self.time_to_squeeze = 1.0
        self.time_to_yield = 3.0

    def get_next_action(self, checking_agent, all_agents, idle_speed=30):
        """
        :param checking_agent: the ego agent for the next action
        :param all_agents: all agents to check collision
        :param idle_speed: the idel_speed for turning under normal model
        :return: "yield", "stop", "normal", "alert", "follow", "yield_squeeze"

        We assume the future trajectories of each agent are public for any other agents.
        You can re-predict the trajectory for each agent (use get_possible_destinations() and destinations_to_paths_batch())
        but under the same motion planning, the predicted trajectory would be the same as the original one.
        So no need to recompute.
        """

        # TODO: ignore opposite direction forward driving cars to speed up
        checking_agent_trajectory = checking_agent.trajectory
        self.etc = None
        self.next_collision_pt = None
        self.max_etc_for_follow_collision = None
        mss_to_pff = self.scale / self.frame_rate / self.frame_rate
        next_action = "normal"
        vigilant = False

        if checking_agent.points_before_next_stop_line > int(
                                self.distance_to_stopline * checking_agent.motion_planner.sample_rate * self.frame_rate):
            return next_action

        for target_agent in all_agents:
            distance = euclidean_distance(pt1=(target_agent.x, target_agent.y),
                                          pt2=(checking_agent.x, checking_agent.y))
            if distance < self.distance_to_slow * self.scale * 1.2:
                vigilant = True
            else:
                vigilant = False

            if not target_agent == checking_agent and len(target_agent.trajectory) > 1:
                target_trajectory = target_agent.trajectory
                target_direction = target_agent.direction_in_text
                checking_direction = checking_agent.direction_in_text
                # collision_dist = (checking_agent.width + target_agent.width)/2
                last_etc = self.etc
                is_closet = False
                collision = self.check_path_collision(trajectory_1=checking_agent_trajectory,
                                                      trajectory_2=target_trajectory,
                                                      v1=max(idle_speed * 0.8, checking_agent.vx),
                                                      v2=target_agent.vx,
                                                      skip_rate=self.skip_rate,
                                                      start_from=max(checking_agent.points_before_next_stop_line, 0))
                if self.etc is not None:
                    if last_etc is None or self.etc < last_etc:
                        is_closet = True

                if is_closet:
                    if collision == "follow":
                        self.follow_agent = target_agent
                        next_action = "follow"
                    else:
                        self.follow_agent = None
                        if collision:
                            # possible collision in the future
                            turning = ["R", "L"]
                            # v0.3 for 4 ways stop signs
                            checking_from_intersection = checking_agent.spawn_position[0]
                            target_from_intersection = target_agent.spawn_position[0]

                            # dist = manhattan_distance((checking_agent.x, checking_agent.y), (target_agent.x, target_agent.y))
                            if (not target_direction in turning) and checking_agent.direction_in_text in turning:
                                # yield to forward vehicles: target forward, checking turning
                                next_action = "yield"
                                continue
                            elif (not checking_direction in turning) and target_direction in turning:
                                # forward vehicles expect yielded by others: checking forward, target turning
                                if target_agent.vx > self.alert_speed * 0.5 or target_agent.a == checking_agent.policy.accelerate_rate * mss_to_pff * 0.8:
                                    # but if the other is still driving fast, forward with cautious
                                    if self.etc < self.time_to_yield * self.frame_rate:
                                        next_action = "yield_squeeze"
                                    else:
                                        next_action = "alert"
                                    continue
                                if target_agent.policy.etc is not None and target_agent.policy.etc < self.time_to_squeeze * self.frame_rate:
                                    # yield to squeezing agents
                                    next_action = "yield_squeeze"
                                    continue
                            elif (not checking_direction in turning) and (not target_direction in turning):
                                # target forward, checking forward for 4 ways stop signs
                                intersection_bias = (target_from_intersection - checking_from_intersection) % 4
                                if intersection_bias == 2:  # NOTE: this logic only works for 4 ways intersection
                                    continue
                                elif (target_from_intersection - checking_from_intersection) < 0 and intersection_bias % 2 == 1:
                                    # yield cars coming from the right
                                    next_action = "yield"
                                    continue
                            elif checking_direction in turning and target_direction in turning:
                                # target turning, checking turning
                                intersection_bias = (target_from_intersection - checking_from_intersection) % 4
                                if (target_from_intersection - checking_from_intersection) < 0 and intersection_bias % 2 == 1:
                                    # yield cars coming from the right
                                    next_action = "yield"
                                    continue

                            # default logic for everything else
                            if target_agent.next_action not in ["yield", "stop"]:
                                # if dist < self.emergency_dist and target_agent.next_action not in ["yield", "stop"]:
                                # if False: # 0.5 < target_agent.vx < 5:
                                #     next_action = "stop"
                                if len(checking_agent_trajectory[0]) / checking_agent.vx > len(
                                        target_trajectory[0]) / target_agent.vx:
                                    next_action = "yield "
                                else:
                                    if not next_action in ["stop", "yield"]:
                                        next_action = "alert"

        if vigilant:
            self.action_delay = 0.4
        else:
            self.action_delay = 0.8
        return next_action

    def check_path_collision(self, trajectory_1, trajectory_2, v1=1, v2=1, skip_rate=15, start_from=0):
        """
        :param trajectory_1:
        :param trajectory_2:
        :param v1:
        :param v2:
        :param skip_rate:
        :param start_from: from n frame to detect
        :return: False, True(collision), "follow"(collision but need to follow)
        """
        v1 += 0.01
        # v2 += 0.01
        v2 = max(v2, 0.3)
        len_traj1 = len(trajectory_1[0])
        len_traj2 = len(trajectory_2[0])
        time_total = int(min(len_traj1 / v1, len_traj2 / v2))
        # skip_rate = skip_rate
        unscaled_size = self.vehicle_size[0] / self.scale, self.vehicle_size[1] / self.scale
        follow = False
        for i in range(time_total - 1):
            if i % skip_rate == 0 and i >= start_from:
                i1 = int(i * v1)
                i2 = int(i * v2)
                if i1 + 1 >= len_traj1 or i2 + 1 >= len_traj2:
                    break
                yaw1 = math.atan2(trajectory_1[0][i1 + 1][1] - trajectory_1[0][i1][1],
                                  trajectory_1[0][i1 + 1][0] - trajectory_1[0][i1][0])
                yaw2 = math.atan2(trajectory_2[0][i2 + 1][1] - trajectory_2[0][i2][1],
                                  trajectory_2[0][i2 + 1][0] - trajectory_2[0][i2][0])
                if check_collision_for_point_in_path(pt1=trajectory_1[0][i1],
                                                     size1=unscaled_size,
                                                     yaw1=yaw1,
                                                     pt2=trajectory_2[0][i2],
                                                     size2=unscaled_size,
                                                     yaw2=yaw2,
                                                     vertical_margin=2):
                    if abs(normalize_angle(yaw1 - yaw2)) < math.pi / 180 * 45:
                        if len_traj2 < len_traj1:
                            # if target agent is ahead of checking agent and its the nearest one, then follow it
                            # if target agent is behind the checking agent, continue checking remaining trajectory for collision
                            if self.max_etc_for_follow_collision is None or len_traj2 > self.max_etc_for_follow_collision:
                                self.max_etc_for_follow_collision = len_traj2
                                follow = True

                    if self.etc is None or i < self.etc:
                        self.etc = i
                        self.next_collision_pt = trajectory_1[0][i1]
                    if follow:
                        return "follow"
                    else:
                        return True

        return False
