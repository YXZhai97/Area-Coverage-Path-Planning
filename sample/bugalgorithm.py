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


def tangent_bug(start_point, goal_point, env_map):
    rs = 5
    step_length = 0.5
    state = np.zeros((Iteration, 2))
    state[0]=start_point
    x_n = len(env_map[0])  # number of col
    y_n = len(env_map)  # number of row
    obs_map = np.zeros((y_n, x_n))
    mode = 0  # motion to goal , mode=1-> boundary follow

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

    def update_obsmap(time):
        p = state[time]
        for row in range(y_n):
            for col in range(x_n):
                grid_point = get_center(row, col)
                distance = np.linalg.norm(p - grid_point)
                if distance <= rs and env_map[row, col] == 1:
                    obs_map[row, col] = -1



    def chek_intersection(state,goal):
        visit = set()
        angle_sets=[]
        dx = goal[0] - state[0]
        dy = goal[1] - state[1]
        goal_state_angle = atan2(dy, dx) * 180 / pi



        def bfs(r,c):
            angle=np.zeros(3)
            q=collections.deque(r,c)
            visit.add((r,c))
            q.append((r,c))

            while q:
                rr,cc = q.popleft()
                directions=[[1,0],[-1,0],[0,1],[0,-1]]

                center = get_center(rr,cc)
                angle=np.vstack((angle,get_angle(state,center))


                for dr,dc in directions:
                    r,c=rr+dr,cc+dc
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
            angle_range=angle_set[:,0]
            max_angle=max(angle_range)
            min_angle=min(angle_range)
            if max_angle>goal_state_angle>min_angle:
                return angle_set


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



    def get_heuristic_goal(angle_set):
        angle_range=angle_set[:,0]
        max_angle=max(angle_range)
        min_angle=min(angle_range)
        max_index=angle_range.index(max_angle)
        min_index=angle_range.index(min_angle)
        end_point1=angle_set[max_index,1:]
        end_point2=angle_set[min_index,1:]
        heuristic_distance1=np.linalg.norm(state-end_point1)+np.linalg.norm(end_point1-goal_point)
        heuristic_distance2 = np.linalg.norm(state - end_point2) + np.linalg.norm(end_point2 - goal_point)

        if heuristic_distance1>heuristic_distance2:
            return end_point2
        else:
            return end_point1



    def go_straight():
        pass

    def boundary_follow():
        pass
