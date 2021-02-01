from util import *
import math
import numpy as np

class DefaultDynamicModel:
    def __init__(self, dt=0.1):
        self.dt = dt
        print("Dynamic Model Init")

    def update(self, agent, throttle, delta):
        delta = np.clip(delta, -agent.max_steer, agent.max_steer)
        agent.x = agent.x + agent.vx * math.cos(agent.yaw) * self.dt - agent.vy * math.sin(agent.yaw) * self.dt
        agent.y = agent.y + agent.vx * math.sin(agent.yaw) * self.dt + agent.vy * math.cos(agent.yaw) * self.dt
        agent.yaw += agent.omega * self.dt
        agent.yaw = normalize_angle(agent.yaw)
        Ffy = -agent.Cf * math.atan2(((agent.vy + agent.Lf * agent.omega) / agent.vx - delta), 1.0)
        Fry = -agent.Cr * math.atan2((agent.vy - agent.Lr * agent.omega) / agent.vx, 1.0)
        R_x = agent.c_r1 * agent.vx
        F_aero = agent.c_a * agent.vx ** 2
        F_load = F_aero + R_x
        agent.vx += (throttle - Ffy * math.sin(delta) / agent.m - F_load/agent.m + agent.vy * agent.omega) * self.dt
        agent.vy += (Fry / agent.m + Ffy * math.cos(delta) / agent.m - agent.vx * agent.omega) * self.dt
        agent.omega += (Ffy * agent.Lf * math.cos(delta) - Fry * agent.Lr) / agent.Iz * self.dt

class DefaultKinematicModel:
    def __init__(self, dt=1.0):
        self.dt = dt
        print("Dynamic Model Init")

    def update_nl(self, agent, throttle, delta):
        # x to up, y to right
        delta = np.clip(delta, -agent.max_steer, agent.max_steer)
        agent.x = agent.x + agent.vx * math.sin(agent.yaw) * self.dt + agent.vy * math.cos(agent.yaw) * self.dt
        agent.y = agent.y + agent.vx * math.cos(agent.yaw) * self.dt - agent.vy * math.sin(agent.yaw) * self.dt
        agent.yaw += agent.omega * self.dt
        agent.yaw = normalize_angle(agent.yaw)
        Ffy = -agent.Cf * math.atan2(((agent.vy + agent.Lf * agent.omega) / agent.vx - delta), 1.0)
        Fry = -agent.Cr * math.atan2((agent.vy - agent.Lr * agent.omega) / agent.vx, 1.0)
        R_x = agent.c_r1 * agent.vx
        F_aero = agent.c_a * agent.vx ** 2
        F_load = F_aero + R_x
        agent.vx += (throttle - Ffy * math.sin(delta) / agent.m - F_load/agent.m + agent.vy * agent.omega) * self.dt
        agent.vy += (Fry / agent.m + Ffy * math.cos(delta) / agent.m - agent.vx * agent.omega) * self.dt
        agent.omega += (Ffy * agent.Lf * math.cos(delta) - Fry * agent.Lr) / agent.Iz * self.dt

    def update(self, agent, throttle, delta):
        """
        Update the state of the vehicle.
        Stanley Control uses bicycle model.
        :param a: (float) Acceleration
        :param delta: (float) Steering
        """
        delta = np.clip(delta, -agent.max_steer, agent.max_steer)
        dt = self.dt
        agent.x += agent.vx * np.cos(agent.yaw) * dt
        agent.y += agent.vx * np.sin(agent.yaw) * dt
        agent.yaw += agent.vx / agent.Lf * np.tan(delta) * dt
        agent.yaw = normalize_angle(agent.yaw)
        agent.vx += throttle * dt
        agent.a = throttle
