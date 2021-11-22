"""
The Tangent bug path planning algorithm
The robot has limited sensing range and grid map
Author: Yixing Zhai
Date: 17.11.2021

"""
import matplotlib.pyplot as plt

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
    time=0
    mode = 0  # mode=0 motion to goal, mode=1 boundary follow
    # get the intersection curve and endpoints

    # start the main while loop
    while True:
        cur_state=state[time]
        is_intersect, end_points, curve=get_curve(obstacles,cur_state,goal_point,rs)

        if is_intersect:
            temp_goal=get_heuristic_goal(cur_state,goal_point,end_points)
        else:
            temp_goal=goal_point

        if mode==0: # go straight to goal
            pass









def get_curve(obstacles, cur_state, goal_point, rs):
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
    cur_x = cur_state[0]
    cur_y = cur_state[1]
    num_iter = 360
    angle_space = np.linspace(-pi, pi, num_iter)
    angle_step = 2 * pi / num_iter

    circle_scanned = np.zeros((num_iter, 2))
    circle_occupied = np.zeros(num_iter)

    end_points=[]
    is_intersect = False
    intersect_end_points=[]
    intersect_curve=[]

    for i in range(num_iter):
        len_close = rs
        i_angle = angle_space[i]
        for obstacle in obstacles:
            num_points = len(obstacle[0])
            for n_p in range(num_points):
                obs_x = obstacle[0, n_p]
                obs_y = obstacle[1, n_p]
                obs_point = np.array([obs_x, obs_y])
                # print(obs_point)
                angle_obs = atan2(obs_y - cur_y, obs_x - cur_x)

                if abs(angle_obs - i_angle) < angle_step:
                    cur_to_obs = np.linalg.norm(cur_state - obs_point)
                    print(cur_to_obs)
                    if cur_to_obs < len_close:
                        circle_scanned[i] = obs_point
                        len_close=cur_to_obs
                        if circle_occupied[i] == 0:
                            circle_occupied[i] = 1

    continue_list = np.zeros(num_iter)
    for i in range(num_iter):
        if i ==0:
            continue_list[i]=circle_occupied[i]+circle_occupied[i+1]+circle_occupied[-1]
        elif i==num_iter-1:
            continue_list[i]=circle_occupied[i-1]+circle_occupied[0]+circle_occupied[i]
        else:
            continue_list[i]=circle_occupied[i-1]+circle_occupied[i+1]+circle_occupied[i]
        if continue_list[i]==2:
            end_points.append(circle_scanned[i])

    end_point_num = len(end_points)
    if end_point_num == 2:
        is_intersect = check_intersection(cur_state, goal_point, end_points)
        if is_intersect:
            intersect_end_points=end_points

    else:
        for i in range(end_point_num):
            if i ==0:
                ep=[end_points[0],end_points[-1]]
                is_intersect=check_intersection(cur_state,goal_point,ep)
            elif i%2==1 and i!=end_point_num-1:
                ep=[end_points[i],end_points[i+1]]
                is_intersect = check_intersection(cur_state, goal_point, ep)
            if is_intersect:
                intersect_end_points=ep
                break

    return is_intersect, intersect_end_points, circle_scanned


def check_intersection(cur_p, goal_p, end_points):
    """
    Args:
        cur_p: np array
        goal_p: np array
        end_points: list of list [[p1x,p1y],[p2x,p2y]]
    Returns:
        is_intersect: True or Flase
    """
    o1 = end_points[0]
    o2 = end_points[1]
    if cur_p[0] != goal_p[0]:
        multi = (o1[1] - get_line(cur_p, goal_p, o1[0])) * (o2[1] - get_line(cur_p, goal_p, o2[0]))
    else:
        multi = (o1[0] - cur_p) * (o2[0] - goal_p[0])

    if multi > 0:
        is_intersect = False
    else:
        is_intersect = True

    return is_intersect


def get_line(p1, p2, x):
    """
    Args:
        p1: point 1
        p2: point 2
        x: x coordinate
    Returns:
        y coordinate with respect to x
    """
    x1, y1 = p1
    x2, y2 = p2
    y = (x - x1) / (x2 - x1) * (y2 - y1) + y1

    return y


def get_heuristic_goal(cur_state,goal_point,end_points):
    heuristic_d0=np.linalg.norm(cur_state-end_points[0])+np.linalg.norm(end_points[0]-goal_point)
    heuristic_d1=np.linalg.norm(cur_state-end_points[1])+np.linalg.norm(end_points[1]-goal_point)

    if heuristic_d0>heuristic_d1:
        oi=end_points[1]
    else:
        oi=end_points[0]

    return oi


def go_straight(cur_state, temp_goal, step_len):

    dy=temp_goal[1]-cur_state[1]
    dx=temp_goal[0]-cur_state[0]
    angle=atan2(dy,dx)
    distance=np.linalg.norm(cur_state-temp_goal)
    if distance>step_len:
        next_x=cur_state[0]+cos(angle)*step_len
        next_y=cur_state[1]+sin(angle)*step_len

    next_state=np.array([next_x,next_y])

    return next_state




def boundary_follow():
    pass

def check_along():
    pass

def get_closest_point(cur_state,curve):
    pass



mymap = EnvMap(300, 300, 1)
mymap.add_polygon([100, 100, 160, 180, 160, 250, 70, 280])
mymap.add_circle(172, 115, 35)
mymap.show_map()
obs = mymap.obstacles
state = np.array([160, 140])
goal_point = np.array([50, 50])
occu, scanned,continue_list,end_points = get_curve(obs, state, goal_point, 10)
# print(occu)
# print(scanned)
plt.scatter(scanned[:, 0], scanned[:, 1], s=5, c='r')

# print(get_line([1,2],[8,0],3))
