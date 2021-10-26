import numpy as np
import math
import matplotlib.pyplot as plt
from robot import *
import methods as m
from env_map import *
import global_value

"""
The main script for path planning 
"""

# robot initialization
# define three robot and initialize the state and target
define_robot(3)
for robot in gv.robotList:
    robot.random_init_state()
    robot.random_init_target()

    print(robot.state[0])

show_robot(gv.robotList)

# environment initialization
# define the 2D map with length 100 width 100 and grid length 1
mymap = EnvMap(300,300,1)

# add a circle
mymap.add_circle(100,150,30)
# add a triangle
# mymap.add_polygon([50,50,70,80,80,30])
mymap.show_map()

# iteration


# state and map update


# plot the path
# 2D animation
# 2D static

