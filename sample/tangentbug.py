"""
The Tangent bug path planning algorithm
The robot has limited sensing range and grid map
Author: Yixing Zhai
Date: 17.11.2021

"""
import numpy as np
from env_map import *
from math import *



def tangent_bug(start_point, goal_point, environment):
    '''
    Args:
        start_point: The initial position of the robot
        goal_point: The goal position
        environment: A gird map with obstacles
    Returns:
        A path
    '''

    state = np.array(start_point) # the state array will be appended
    goal_point = np.array(goal_point)
    obs_map = environment.gridmap
    x_n = environment.x_n
    y_n = environment.y_n
    info_map = np.zeros((y_n,x_n))
    rs=5
    step_len=1
    mode=0 # mode=0 motion to goal, mode=1 boundary follow

    while True:

















