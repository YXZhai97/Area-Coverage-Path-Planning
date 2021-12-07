from env_map import *
from robot import *
from robot_animation import *
import matplotlib.animation as animation
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
mymap.add_polygon([10,10,25,10,25,25,10,25])
mymap.show_map()

# iteration
# monitor the coverage percent
c_percent=np.zeros(gv.Iteration-1)
# todo there is index problem with time+1
for time in range(gv.Iteration-1):
    for robot in robotList:
        # get current robot state
        cur_state=robot.state[time, :2]
        cur_target=robot.target[time]
        # update information map and motion-mode
        robot.update_info_map(time, mymap)
        robot.update_motion_mode(time, mymap, cur_target)
        # get motion mode
        motion_mode=robot.motion_mode
        # update the target based on the motion-mode
        if motion_mode==1:
            new_target=robot.update_target_tangent(time, cur_state, mymap)
            robot.update_state_tangent(time, new_target)
        else:
            new_target=robot.update_target(time)
            robot.update_state(time)

    # merge the infomap of the robots
    coverage_percent=merge_infomap(robotList)
    c_percent[time]=coverage_percent

    # print log info
    print("Time step:", time)
    print("Coverage percent:", coverage_percent)


# plot the coverage percent
show_coverage_percent(c_percent)

# plot the robot target position
figure2=plt.figure('robot target position', figsize=(5,5))
for robot in robotList:
    # plt.plot(robot.state[:, 0], robot.state[:, 1])
    plt.scatter(robot.target[:,0],robot.target[:,1])
plt.xlabel("X coordinate [m]")
plt.ylabel("Y coordinate [m]")
plt.title("The robot path with simulation time %i " %gv.T + ",time step %1.3f s " %gv.step_size +",robot number %i" %gv.robot_number   )
plt.savefig('../image/target12.png')
plt.show()


# plot the robot path
def plot_robot_path(robotList, mymap):

    figure3=plt.figure('robot path ', figsize=(7,7))
    for robot in robotList:
        plt.plot(robot.state[:, 0], robot.state[:, 1])
    for obstacle in mymap.obstacles:
        plt.plot(obstacle[0], obstacle[1], color='k', linewidth=2 )
    plt.xlabel("X coordinate [m]")
    plt.ylabel("Y coordinate [m]")
    plt.title("The robot path with simulation time %i " %gv.T + ",time step %1.3f s " %gv.step_size +",robot number %i" %gv.robot_number   )
    plt.savefig('../image/path12.png')

plot_robot_path(robotList, mymap)

# plot the information map of the robot
figure4=plt.figure('robot information map ', figsize=(10,10))
subfig1=figure4.add_subplot(221)
show_infomap(robotList[0], mymap)
subfig2=figure4.add_subplot(222)
show_infomap(robotList[1], mymap)
subfig3=figure4.add_subplot(223)
show_infomap(robotList[2], mymap)
subfig4=figure4.add_subplot(224)
show_merge_infomap(robotList)

plt.savefig('../image/infomap12.png')



# 2D animation
anim=visualize(robotList)
writervideo = animation.FFMpegWriter(fps=10) # fps is (frames per second)
anim.save('../image/robot_path_animation12.mp4', writer=writervideo)
# 2D static



