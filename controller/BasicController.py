"""

"""
from util import *

class PIDController:
    def __init__(self, d_p=.5, d_i=.01, d_d=0, y_p=5, y_i=1, y_d=0):
        # ignore acceleration
        self.distance_p = d_p
        self.distance_i = d_i
        self.distance_d = d_d
        self.yaw_p = y_p
        self.yaw_i = y_i
        self.yaw_d = y_d
        self.distance = 0
        self.distance_integral = 0
        self.distance_derivative = 0
        self.yaw = 0
        self.yaw_integral = 0
        self.yaw_derivative = 0

    def get_control_signal(self, agent, scale, frame_rate):
        target_pva = agent.trajectory[0]
        current_pva = [(agent.x / scale, agent.y / scale), agent.vx / scale * frame_rate, 0]
        # calculate throttle
        delta_position = manhattan_distance(target_pva[0], current_pva[0])
        vector_angle = math.atan2(target_pva[0][1] - current_pva[0][1],
                                  target_pva[0][0] - current_pva[0][0])
        delta_angle = abs(normalize_angle(vector_angle - agent.yaw))
        distance = delta_position * math.cos(delta_angle)
        self.distance_integral += distance
        self.distance_derivative = distance - self.distance
        self.distance = distance
        throttle = self.distance_p * distance + self.distance_i * self.distance_integral + self.distance_d * self.distance_derivative

        # calculate delta
        delta_yaw = normalize_angle(vector_angle - agent.yaw)
        self.yaw_integral += delta_yaw
        self.yaw_derivative = delta_yaw - self.yaw
        self.yaw = delta_yaw
        delta = self.yaw_p * delta_yaw + self.yaw_i * self.yaw_integral + self.yaw_d * self.yaw_derivative

        return throttle, delta


