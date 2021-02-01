# Jerk Minimization Trajectory

# fit the poly with {current_state, target_state, t=0}
# get states for each frame with {frequency} in an array
# get states for specific time

from util import *
import numpy as np

class PolynomialTrajectaryGenerator:
    # state = [position, velocity, acceleration]
    # speed as mph
    def __init__(self, current_state, target_state, t=0, speed_limit=50, sample_rate=1):  # sample_rate->1m/s static speed
        self.p_0, self.v_0, self.a_0 = self.decouple_axis(current_state)
        self.p_t, self.v_t, self.a_t = self.decouple_axis(target_state)
        self.current_v = current_state[1]
        self.sample_rate = sample_rate
        self.t = t
        self.speed_limit = speed_limit
        self.A = self.fit_poly()

    def decouple_axis(self, state):
        p, v, a, theta = state
        x, y = p
        return (x, y), (v*math.sin(theta), -v*math.cos(theta)), (a*math.sin(theta), -a*math.cos(theta))

    def fit_poly(self):
        if True:
            # generate a t
            lower_t = float(manhattan_distance(self.p_0, self.p_t)) / self.current_v
            upper_t = float(euclidean_distance(self.p_0, self.p_t)) / self.current_v
            self.t = (lower_t + upper_t) / 2.0

        A = []
        t = self.t
        for i in range(0, 2):
            p_0 = self.p_0[i]
            v_0 = self.v_0[i]
            a_0 = self.a_0[i]

            p_t = self.p_t[i]
            v_t = self.v_t[i]
            a_t = self.a_t[i]

            T = np.matrix([[t**3, t**4, t**5],
                           [3*t**2, 4*t**3, 5*t**4],
                           [6*t, 12*t**2, 20*t**3]])
            S = np.matrix([p_t, v_t, a_t]).transpose()
            C = np.matrix([p_0+v_0*t+0.5*a_0*t**2,
                           v_0+0.5*a_0*t**2,
                           a_0]).transpose()
            A.append(np.linalg.inv(T)*(S-C))
            # print("fit complete at i: ", i, A[i].flat)
        return A

    def get_states(self, frequency):
        if self.t==0:
            print("ERROR, get states before fit")
        t = self.t
        p_array = []
        v_array = []
        a_array = []
        data_rate = frequency * self.current_v / self.sample_rate
        for i in range(0, int(t * data_rate)):
            current_t = float(i)/data_rate
            p, v, a = self.time_to_states(current_t)
            p_array.append(p)
            v_array.append(v)
            a_array.append(a)
        return [p_array, v_array, a_array]

    def time_to_states(self, t):
        A_flat_x = self.A[0].flat
        a_0_x = self.p_0[0]
        a_1_x = self.v_0[0]
        a_2_x = self.a_0[0]*0.5
        a_3_x = A_flat_x[0]
        a_4_x = A_flat_x[1]
        a_5_x = A_flat_x[2]
        p_x = a_0_x+a_1_x*t+a_2_x*t**2+a_3_x*t**3+a_4_x*t**4+a_5_x*t**5
        v_x = a_1_x+2*a_2_x*t+3*a_3_x*t**2+4*a_4_x*t**3+5*a_5_x*t**4
        a_x = 2*a_2_x+6*a_3_x*t+12*a_4_x*t**2+20*a_5_x*t**3

        A_flat_y = self.A[1].flat
        a_0_y = self.p_0[1]
        a_1_y = self.v_0[1]
        a_2_y = self.a_0[1]*0.5
        a_3_y = A_flat_y[0]
        a_4_y = A_flat_y[1]
        a_5_y = A_flat_y[2]
        p_y = a_0_y+a_1_y*t+a_2_y*t**2+a_3_y*t**3+a_4_y*t**4+a_5_y*t**5
        v_y = a_1_y+2*a_2_y*t+3*a_3_y*t**2+4*a_4_y*t**3+5*a_5_y*t**4
        a_y = 2*a_2_y+6*a_3_y*t+12*a_4_y*t**2+20*a_5_y*t**3

        p = (p_x, p_y)
        v = (v_x, v_y)
        a = (a_x, a_y)

        return p, v, a









