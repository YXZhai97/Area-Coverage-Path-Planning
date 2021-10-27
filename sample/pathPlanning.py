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
    print(robot.state[0]) # test the initial state has been passed to the state
    print(robot.target[0])
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
# todo there is index problem with time+1
for time in range(gv.Iteration-1):
    for robot in gv.robotList:
        # update information map
        robot.update_info_map(time)
        # calculate benefit value and target
        robot.update_target(time)
        # update robot state
        robot.update_state(time)


# plot the path
for robot in gv.robotList:
    plt.plot(robot.state[:,0],robot.state[:,1])

plt.savefig('../image/path.png')
plt.show()


# 2D animation
# 2D static

