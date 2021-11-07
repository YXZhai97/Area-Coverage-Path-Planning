from env_map import *
from robot import *
from robot_animation import *
"""
The main script for path planning 
"""

# robot initialization
# define three robot and initialize the state and target
robotList = define_robot(gv.robot_number)

for robot in robotList:
    robot.random_init_state()
    robot.random_init_target()
    print(robot.state[0])  # test the initial state has been passed to the state
    print(robot.target[0])
show_robot(robotList)

# environment initialization
# define the 2D map with length 100 width 100 and grid length 1
mymap = EnvMap(50, 50, 1)
# add a circle
# mymap.add_circle(100, 150, 30)
# add a triangle
# mymap.add_polygon([50,50,70,80,80,30])
mymap.show_map()

# iteration
# monitor the coverage percent
c_percent=np.zeros(gv.Iteration-1)
# todo there is index problem with time+1
for time in range(gv.Iteration-1):
    for robot in robotList:
        # update information map
        robot.update_info_map(time)
        # calculate benefit value and target
        robot.update_target(time)
        # update robot state
        robot.update_state(time)
    # merge the infomap of the robots
    coverage_percent=merge_infomap(robotList)
    c_percent[time]=coverage_percent

    # print log info
    print("Time step:", time)
    print("Coverage percent:", coverage_percent)




# plot the robot target position
figure2=plt.figure('robot target position', figsize=(5,5))
for robot in robotList:
    # plt.plot(robot.state[:, 0], robot.state[:, 1])
    plt.scatter(robot.target[:,0],robot.target[:,1])
plt.savefig('../image/target8.png')
plt.show()


# plot the robot path
figure3=plt.figure('robot path ', figsize=(8,5))
for robot in robotList:
    plt.plot(robot.state[:, 0], robot.state[:, 1])
plt.xlabel("X coordinate [m]")
plt.ylabel("Y coordinate [m]")
plt.title("The robot path with simulation time %i " %gv.T + ",time step %1.3f s " %gv.step_size +",robot number %i" %gv.robot_number   )
plt.savefig('../image/path8.png')


# plot the information map of the robot
figure4=plt.figure('robot information map ', figsize=(10,10))
subfig1=figure4.add_subplot(221)
show_infomap(robotList[0])
subfig2=figure4.add_subplot(222)
show_infomap(robotList[1])
subfig3=figure4.add_subplot(223)
show_infomap(robotList[2])
subfig4=figure4.add_subplot(224)


plt.savefig('../image/infomap8.png')



# 2D animation
anim=visualize(robotList)
# 2D static

# todo the boundary is not modeled, no need to model the boundary
# todo: robot moving out of the boundary, as long as the target points inside the boundary
# todo check the matlab code

