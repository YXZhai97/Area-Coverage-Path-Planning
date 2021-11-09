"""
Implement Tangent Bug Algorithm for Mobile Robot
"""
from env_map import *
import numpy as np

mymap = EnvMap(100, 100, 1)
mymap.add_polygon([50,50,70,80,80,60])
mymap.show_map()
Iteration=40

def tangent_bug(start_point,goal_point,env_map):
    rs=5
    step_length=1
    state=np.zeros((Iteration,2))
    obs_map=np.zeros((100,100))
    mode=0 # motion to goal , mode=1-> boundary follow


    def update_obsmap():
        pass


    def get_endpoints():
        pass


    def go_straight():
        pass


    def boundary_follow():
        pass










