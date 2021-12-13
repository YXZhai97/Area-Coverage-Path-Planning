from robot import *
from env_map import *

mymap = EnvMap(50, 50, 1)
# add a circle
# mymap.add_circle(100, 150, 30)
# add a triangle
mymap.add_polygon([10,10,25,10,25,25,10,25])
mymap.add_circle(35,30,8)
mymap.add_polygon([30,5,40,5,40,15,30,15])
# mymap.show_map()

# robot initialization
# define three robot and initialize the state and target
robotList = define_robot(3)
for robot in robotList:
    robot.random_init_state(mymap)
    robot.random_init_target()
    print(robot.state[0])  # test the initial state has been passed to the state
    print(robot.target[0])
show_robot(robotList, mymap)
