"""
Implement Tangent Bug Algorithm for Mobile Robot
"""
import numpy as np

import global_value as gv
from env_map import *

mymap = EnvMap(100, 100, 1)
mymap.add_polygon([50, 50, 70, 80, 80, 60])
mymap.show_map()
Iteration = 40


def tangent_bug(start_point, goal_point, env_map):
    rs = 5
    step_length = 1
    state = np.zeros((Iteration, 2))
    x_n = len(env_map[0])  # number of col
    y_n = len(env_map)  # number of row
    obs_map = np.zeros((y_n, x_n))
    mode = 0  # motion to goal , mode=1-> boundary follow

    def get_center(row, col):
        x_center = col * gv.grid_length + gv.grid_length / 2
        y_center = row * gv.grid_length + gv.grid_length / 2
        return np.array([x_center, y_center])

    def update_obsmap(time):
        p = state[time]
        for row in range(y_n):
            for col in range(x_n):
                grid_point = get_center(row, col)
                distance = np.linalg.norm(p - grid_point)
                if distance <= rs and env_map[row, col] == 1:
                    obs_map[row, col] = -1

    def get_endpoints():
        pass

    def go_straight():
        pass

    def boundary_follow():
        pass
