import numpy as np
import math
import matplotlib.pyplot as plt
from robot import *
import methods as m
import env_map
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

show_robot(gv.robotList)





# environment initialization


# iteration


# state and map update


# plot the path
# 2D animation
# 2D static

