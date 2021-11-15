"""
Implement Tangent Bug Algorithm for Mobile Robot
"""
import numpy as np
import collections
import global_value as gv
from env_map import *
from math import *

mymap = EnvMap(100, 100, 1)
mymap.add_polygon([50, 50, 70, 80, 80, 60])
mymap.show_map()
Iteration = 40

start_point=np.array([10,20])
goal_point=np.array([90,90])


def tangent_bug(start_point, goal_point, mymap):
    '''
    Args:
        start_point: [x,y] a array
        goal_point: [px,py] a array
        env_map: n by n matrix

    Returns:
        a path
    '''
    rs = 5
    decrease_factor=5
    state = np.array(start_point)
    gridmap=mymap.grid_map
    x_n = len(gridmap[0])  # number of col
    y_n = len(gridmap)  # number of row
    obs_map = np.zeros((y_n, x_n))
    mode = 0  # motion to goal , mode=1-> boundary follow
    time=0
    intersect_angle_sets=np.array([])

    """
    steps to calculate endpoints
    1. update obs_map
    2. get island 
    3. check the intersection island
        * distance to m line smaller than grid length
        * between state and goal 
    4. get the max and min angle of the island points, store them as endpoints 
    5. calculate the heuristic distance of these two points 
    
    """

    def get_center(row, col):
        x_center = col * gv.grid_length + gv.grid_length / 2
        y_center = row * gv.grid_length + gv.grid_length / 2

        return np.array([x_center, y_center])

    def get_angle(p1,p2):
        '''
        Args:
            p1: [x1,y1] of the first point
            p2: [x2,y2] of the second point
        Returns:
            angle of the line, and position of p2
        '''
        dx=p2[0]-p1[0]
        dy=p2[1]-p1[1]
        angle=atan2(dy,dx)*180/pi

        return [angle,p2[0],p2[1]]

    def update_obsmap(time):
        p = state[time]
        flag=0
        for row in range(y_n):
            for col in range(x_n):
                grid_point = get_center(row, col)
                distance = np.linalg.norm(p - grid_point)
                if distance <= rs and gridmap[row, col] == 1:
                    obs_map[row, col] = -1
                    flag=1
        # flag=1:obstacle nearby
        return flag



    def chek_intersection(state,goal):
        visit = set()
        angle_sets=[]
        dx = goal[0] - state[0]
        dy = goal[1] - state[1]
        goal_state_angle = atan2(dy, dx) * 180 / pi
        intersection=0

        def bfs(r, c):
            angle=np.zeros(3)
            q=collections.deque()
            visit.add((r,c))
            q.append((r,c))
            while q:
                rr,cc = q.popleft()
                center = get_center(rr,cc)
                angle=np.vstack((angle,get_angle(state,center)))
                directions = [[1, 0], [-1, 0], [0, 1], [0, -1]]
                for dr,dc in directions:
                    r,c = rr+dr,cc+dc
                    if (r in range(y_n) and
                        c in range(x_n) and
                        obs_map[r,c]==-1 and
                        (r,c) not in visit):
                        q.append((r,c))
                        visit.add((r,c))

            return angle[1:]


        for row in range(y_n):
            for col in range(x_n):
                center = get_center(row, col)
                distance = np.linalg.norm(center-state)
                if obs_map[row, col] == -1 and distance <= rs and (row, col) not in visit:
                    angle_sets.append(bfs(row, col))

        for angle_set in angle_sets:
            print(angle_set)
            angle_range=angle_set[:,0]
            max_angle=max(angle_range)
            min_angle=min(angle_range)
            if max_angle>goal_state_angle>min_angle:
                intersection=1
                nonlocal intersect_angle_sets
                intersect_angle_sets=np.array(angle_set)
                print(intersect_angle_sets)
                break
        if intersection==0:
            mode=0 # go straight
        else:
            mode=1 # follow boundary

        return mode


    def get_heuristic_goal(angle_set):
        print("angle set",angle_set)
        angle_range=angle_set[:,0]
        print("angle range", angle_range)

        max_angle=np.max(angle_range)

        min_angle=np.min(angle_range)
        print("min angle",min_angle)

        max_index=np.argmax(angle_range)
        min_index=np.argmin(angle_range)
        print("min index",min_index)

        end_point1=angle_set[max_index,1:]
        print("end_point1",end_point1)
        end_point2=angle_set[min_index,1:]

        heuristic_distance1=np.linalg.norm(state-end_point1)+np.linalg.norm(end_point1-goal_point)
        heuristic_distance2 = np.linalg.norm(state - end_point2) + np.linalg.norm(end_point2 - goal_point)

        if heuristic_distance1>heuristic_distance2:
            return end_point2
        else:
            return end_point1

    # The main loop
    while time<50:
        obs_nearby = update_obsmap(time)
        if obs_nearby:
            mode=chek_intersection(state[time],goal_point)
            print(mode)
            print(intersect_angle_sets)
            if mode:
                temp_goal=get_heuristic_goal(intersect_angle_sets)
            else:
                temp_goal=goal_point
        else:
            temp_goal=goal_point
        print("goal_temp", temp_goal)
        d_state=(temp_goal-state[time])/decrease_factor
        print(d_state)
        new_state=state[time]+d_state
        print(new_state)
        print(state)
        state=np.vstack((state,new_state))

        # check break out condition
        distance=np.linalg.norm(state[time]-goal_point)
        if distance<rs:
            break
        else:
            time+=1

    return state



start_point=[30,20]
goal_point=np.array([80,80])
state=tangent_bug(start_point,goal_point,mymap)
plt.plot(state[:,0],state[:,1])









