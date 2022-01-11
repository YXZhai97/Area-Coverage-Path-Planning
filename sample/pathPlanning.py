
from robot import *
from robot_animation import *
import matplotlib.animation as animation
from floodFill import flood_fill
import tikzplotlib
"""
The main script for path planning 
"""

# environment initialization
# define the 2D map with length 100 width 100 and grid length 1
mymap = EnvMap(gv.x_bound, gv.y_bound, gv.grid_length)
# add a circle
# mymap.add_circle(15, 15, 3)
# mymap.add_circle(40, 20, 4)
# mymap.add_circle(20, 30, 8)

# add a triangle
# mymap.add_polygon([10,10,25,10,25,25,10,25])
mymap.add_polygon([15,15,20,15,20,20,15,20])
mymap.add_circle(5,5,3)

# mymap.show_map()

# robot initialization
# define robots
robotList = define_robot(gv.robot_number)

# set the initial state and target randomly or manually
random=0
if random:
    # initialize the state and target automatically and randomly
    for robot in robotList:
        robot.random_init_state(mymap)
        robot.random_init_target()
        print(robot.state[0])  # test the initial state has been passed to the state
        print(robot.target[0])
else:
    # initialize the robot manually
    # the number of robot is set in global_value.py file
    robot1=robotList[0]
    robot1.set_init_state(15,22)
    robot1.set_init_target(17,19)
    #
    robot2=robotList[1]
    robot2.set_init_state(21,18)
    robot2.set_init_target(19,19)
    #
    # robot3=robotList[2]
    # robot3.set_init_state(10,10)
    # robot3.set_init_target(12,14)

# show the robot initial position and environment map
show_robot(robotList, mymap)

# iteration
# monitor the coverage percent
c_percent=np.zeros(gv.Iteration-1)
# todo there is index problem with time+1

for time in range(gv.Iteration-1):
    for robot in robotList:
        # get current robot state
        cur_state=robot.state[time, :2]
        # update information map and motion-mode
        robot.update_info_map(time, mymap)
        # if robot.motionmode==0:
        #     robot.update_target(time)
        # robot.update_target(time)
        cur_target = robot.target[time]
        if robot.motion_mode==0:
            robot.update_motion_mode(time, mymap, cur_target)
        # update the target based on the motion-mode
        if robot.motion_mode==1 and not robot.inside_tangent_planner :
            target_t=robot.target[time]
            print("start boundary follow")
            print("motion-mode:", robot.motion_mode)
            targets, boundary_follow_finished, followed_curve=robot.tangent_bug_planner(time, mymap)
            # if boundary_follow_finished:
            #     robot.followed_curve.remove([0, 0])
            #     sr, sc = get_middle_point(robot.followed_curve)
            #     print("sr,sc:", sr, sc)
            #     obstacle_color = -1
            #     robot.tarobsmap = flood_fill(robot, sr, sc, obstacle_color)
            cur_target = robot.get_target_from_tangent(time)
            robot.update_state_tangent(time, cur_target)

        elif robot.inside_tangent_planner and time<robot.tangent_end_time-1:
            cur_target=robot.get_target_from_tangent(time)
            if robot.check_bug_neighbour(time):
                robot.reset_target(time)
                robot.update_state(time)
                robot.reset_tangent()
                print("robot meet and stop boundary following")
            else:
                cur_target = robot.get_target_from_tangent(time)
                robot.update_state_tangent(time, cur_target)
        elif not robot.boundary_follow_finished and time==robot.tangent_end_time-1:
            if robot.check_bug_neighbour(time):
                robot.reset_target(time)
                robot.update_state(time)
                robot.reset_tangent()
                print("robot meet and stop boundary following")
            else:
                cur_target = robot.get_target_from_tangent(time)
                robot.update_state_tangent(time, cur_target)

        elif robot.boundary_follow_finished and time==robot.tangent_end_time-1:
            cur_target = robot.get_target_from_tangent(time)
            robot.update_state_tangent(time, cur_target)
            robot.reset_tangent()
            robot.followed_curve.remove([0,0])
            sr, sc = get_middle_point(robot.followed_curve)
            print("sr,sc:", sr, sc)

            obstacle_color = -1
            robot.tarobsmap = flood_fill(robot, sr, sc, obstacle_color)
            print("flood fill is done")
        else:
            robot.update_target(time)
            robot.update_state(time)

    # merge the infomap of the robots
    coverage_percent=merge_infomap(robotList)
    c_percent[time]=coverage_percent

    # print log info
    print("Time step:", time)
    print("Coverage percent:", coverage_percent)
    if coverage_percent>=0.98 :
        print("Area Covered successfully with 98%")
        break

###############################################

# plot the coverage percent
# todo write the plot functions in separate file
show_coverage_percent(c_percent)

# plot the robot target position
figure2=plt.figure('robot target position', figsize=(5,5))
for robot in robotList:
    # plt.plot(robot.state[:, 0], robot.state[:, 1])
    plt.scatter(robot.target[:,0],robot.target[:,1])
for obstacle in mymap.obstacles:
    plt.plot(obstacle[0], obstacle[1])
plt.xlim(0, gv.x_bound)
plt.ylim(0, gv.y_bound)
plt.axis('equal')
plt.xlabel("X coordinate [m]")
plt.ylabel("Y coordinate [m]")
plt.title("The robot path with simulation time %i " %gv.T + ",time step %1.3f s " %gv.step_size +",robot number %i" %gv.robot_number   )
plt.savefig('../image/target32.png')
plt.show()


# plot the robot path


figure3=plt.figure('robot path ', figsize=(7,7))
for robot in robotList:
    plt.plot(robot.state[:time, 0], robot.state[:time, 1])
for obstacle in mymap.obstacles:
    plt.plot(obstacle[0], obstacle[1], color='k', linewidth=2 )
plt.xlim(0, gv.x_bound)
plt.ylim(0, gv.y_bound)
plt.axis('equal')
plt.xlabel("X coordinate [m]")
plt.ylabel("Y coordinate [m]")
plt.title("The robot path with simulation time %i " %gv.T + ",time step %1.3f s " %gv.step_size +",robot number %i" %gv.robot_number   )
plt.savefig('../image/path32.png')
tikzplotlib.save("../tex_files/path_32.tex")



# 2D animation
anim=visualize(robotList, mymap,time)
writervideo = animation.FFMpegWriter(fps=10) # fps is (frames per second)
anim.save('../image/robot_path_animation32.mp4', writer=writervideo)

# plot the information map of the robot
figure4=plt.figure('robot information map ', figsize=(10,10))
subfig1=figure4.add_subplot(221)
show_infomap(robotList[0], mymap)
subfig2=figure4.add_subplot(222)
show_infomap(robotList[1], mymap)
# subfig3=figure4.add_subplot(223)
# show_infomap(robotList[2], mymap)
subfig4=figure4.add_subplot(224)
show_merge_infomap(robotList)

plt.savefig('../image/infomap32.png')







