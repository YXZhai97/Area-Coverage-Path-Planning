"""
The Tangent bug path planning algorithm
The robot has limited sensing range and grid map
Author: Yixing Zhai
Date: 17.11.2021

"""
import matplotlib.pyplot as plt
import numpy as np

from env_map import *


def tangent_bug(start_point, goal_point, my_map):
    '''
    Args:
        start_point: The initial position of the robot
        goal_point: The goal position
        my_map: A gird map with obstacles
    Returns:
        A path
    '''

    state = np.array(start_point)  # the state array will be appended
    goal_point = np.array(goal_point)
    obs_map = my_map.gridmap
    obstacles = my_map.obstacles
    x_n = my_map.x_n
    y_n = my_map.y_n
    info_map = np.zeros((y_n, x_n))
    rs = 5
    step_len = 1
    mode = 0  # mode=0 motion to goal, mode=1 boundary follow

    # get the intersection curve and endpoints


def get_curve(obstacles, cur_state, goal_point,rs):
    """
    Args:
        obstacles: The total obstacles
        cur_state: the current state
        goal_point: the goal point
        rs: sensor range
    Returns:
        a,b,c
    """
    # number of obstacles defined
    num_obs = len(obstacles)
    cur_x=cur_state[0]
    cur_y=cur_state[1]
    num_iter=360
    angle_space=np.linspace(-pi, pi, num_iter)
    angle_step=2*pi/num_iter

    circle_scanned=np.zeros((num_iter,2))
    circle_occupied=np.zeros(num_iter)

    for i in range(num_iter):
        len_close=rs
        i_angle=angle_space[i]
        for obstacle in obstacles:
            num_points=len(obstacle[0])
            for n_p in range(num_points):
                obs_x=obstacle[0,n_p]
                obs_y=obstacle[1,n_p]
                obs_point=np.array([obs_x,obs_y])
                # print(obs_point)
                angle_obs=atan2(obs_y-cur_y,obs_x-cur_x)

                if abs(angle_obs-i_angle)<angle_step:
                    cur_to_obs = np.linalg.norm(cur_state-obs_point)
                    print(cur_to_obs)
                    if cur_to_obs < len_close:
                        circle_scanned[i]=obs_point
                        if circle_occupied[i]==0:
                            circle_occupied[i]=1

    return circle_occupied, circle_scanned


mymap = EnvMap(300, 300, 1)
mymap.add_polygon([100,100,160,180,160,250,70,280])
mymap.add_circle(172,115,35)
mymap.show_map()
obs=mymap.obstacles
state=np.array([164,178])
goal_point=np.array([120,190])
occu,scanned=get_curve(obs,state,goal_point,10)
print(occu)
print(scanned)
plt.scatter(scanned[:,0],scanned[:,1],s=5,c='r')












