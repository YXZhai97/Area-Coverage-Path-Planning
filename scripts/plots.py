
import tikzplotlib
import matplotlib.pyplot as plt
import numpy as np
from global_value import *
from robot import show_merge_infomap, show_infomap
from matplotlib import animation
from save_value import load_variavle
def plot_target_position(robotList, mymap, time):
    """
    This function plots the target points of the robot
    Args:
        robotList: a list of robot object
        mymap: grid map
        time: simulation time

    Returns: save the target points as scatter plot

    """
    plt.figure('robot target position', figsize=(5, 5))
    for robot in robotList:
        plt.scatter(robot.target[:time, 0], robot.target[:time, 1])

    for obstacle in mymap.obstacles:
        plt.plot(obstacle[0], obstacle[1])

    plt.xlim(0, x_bound)
    plt.ylim(0, y_bound)
    plt.axis('equal')
    plt.xlabel("X coordinate [m]")
    plt.ylabel("Y coordinate [m]")
    plt.title(
        "The robot path with iteration steps: %i " % time + ",time step: %1.2f s " % step_size + ",robot rs %1.2f" % rs)
    plt.savefig('../image/target_concave_rc5.png')


def plot_coverage_percent(c_percent, time):
    '''
    This function plots the coverage percent
    Args:
        c_percent: a list of coverage_percent number
        time: the simulation step_number
    Returns:
        plot the coverage percent changes
    '''
    figure = plt.figure('Coverage percent', figsize=(5, 5))
    time_steps = np.linspace(0, step_size*time, time+1)
    plt.plot(time_steps, c_percent[:time+1])
    plt.xlabel("Simulation time")
    plt.ylabel("Coverage percent")
    plt.title("Area Coverage Percent")
    plt.savefig('../image/coverage_percent_many_circles.png')
    # tikzplotlib.save("../tex_file/coverage_percent_concave_rc8.tex")


def plot_robot_path(robotList, mymap, time):

    """
    This function plots the robot path and obstacles
    Args:
        robotList: a list of robot object
        mymap: the grid map
        time: the total simulation time

    Returns: save path image files

    """
    path_color = ['g', 'r', 'b','c','m']
    figure= plt.figure('robot path ', figsize=(6, 6))
    show_merge_infomap(robotList)
    # plot the robot path
    for robot,color in zip(robotList,path_color):
        plt.plot(robot.state[:time, 0], robot.state[:time, 1], color=color, linewidth=1.5)

    # plot the obstacles
    for obstacle in mymap.obstacles:
        plt.plot(obstacle[0], obstacle[1], color='k', linewidth=3)

    # set the x-y axis limits
    plt.xlim(0, x_bound)
    plt.ylim(0, y_bound)
    plt.axis('equal')

    # set label and title
    plt.xlabel("X coordinate [m]")
    plt.ylabel("Y coordinate [m]")
    plt.title(
        "Iteration steps: %i " % time + ",time step: %1.2f s " % step_size + ",robot rc: %1.2f m" % rc+",robot rs: %1.2f m" % rs)

    # save png file and tex file
    plt.savefig('../image/path_many_circles.png')
    # tikzplotlib.save("../tex_file/path_concave_rc8.tex")


def plot_robot_infomap(robotList, mymap):

    figure=plt.figure('robot information map ', figsize=(10, 10))
    subfig1 = figure.add_subplot(221)
    show_infomap(robotList[0], mymap)
    # subfig2 = figure.add_subplot(222)
    # show_infomap(robotList[1], mymap)
    # subfig3=figure.add_subplot(223)
    # show_infomap(robotList[2], mymap)
    # subfig4 = figure.add_subplot(224)
    # show_merge_infomap(robotList)
    plt.savefig('../image/infomap_concave_rs4.png')


def plot_path_on_infomap(robot_path_shot_list, merged_infomap_shot_list, mymap ):
    """
    plot robot path and infomap togather, at different coverage percent:
    15%, 30%, 45%, 60%, 75% 90%
    Args:
        robot_path_shot_list: a list of a list of robots' state list
        mymap: 2D grid map

    Returns: plot image

    """
    figure = plt.figure('robot path at different time ', figsize=(15, 10))
    coverage_percent_list=[15,30,45,60,75,90]
    path_color=['g','r','b']
    for position in range(6):
        percent=coverage_percent_list[position]
        figure.add_subplot(2,3,position+1)
        map_data_3d=merged_infomap_shot_list[position]
        plt.imshow(map_data_3d, origin='lower')

        for path_i, color in zip(robot_path_shot_list[position], path_color):
            plt.plot(path_i[:,0], path_i[:,1], color=color, linewidth=1.5)
            plt.scatter(path_i[-1,0], path_i[-1,1], color=color)

        time=len(path_i)*step_size

        # plot the obstacles
        for obstacle in mymap.obstacles:
            plt.plot(obstacle[0], obstacle[1], color='k', linewidth=2)

        # add title and labels
        plt.xlim(0, x_bound)
        plt.ylim(0, y_bound)
        plt.axis('equal')

        # set label and title
        plt.xlabel("x Position [m]")
        plt.ylabel("y Position [m]")
        plt.title("Coverage Percent: %i " % percent + "%"+" at time %1.1f s" % time )

    plt.savefig('../image/path_on_infomap_many_circles.png')
    # tikzplotlib.save("../tex_file/path_on_map_concave_rc8.tex")


def single_path_on_infomap(robotList, mymap, time):

    figure = plt.figure('with flood fill', figsize=(7, 7))

    show_infomap(robotList[0], mymap)
    for robot in robotList:
        plt.plot(robot.state[:time, 0], robot.state[:time, 1],color='g', linewidth=1.5)
        plt.scatter(robot.state[time-1,0] , robot.state[time-1,1],color='g')

    for obstacle in mymap.obstacles:
        plt.plot(obstacle[0], obstacle[1], color='k', linewidth=2)

    plt.xlim(0, x_bound)
    plt.ylim(0, y_bound)
    plt.axis('equal')

    # set label and title
    plt.xlabel("x Position [m]")
    plt.ylabel("y Position [m]")
    plt.title("Circumnavigation without Flood-Fill")
    plt.savefig('../image/single_robot_with_floodfill.png',dpi=200)






def snapshot_path_map(robotList,time):
    """
    save the merged infomap and robot path at time step t
    Args:
        time:

    Returns: a list of robot state, merged infomation map

    """
    robot_path_shot=[]
    for r in robotList:
        robot_path_shot.append(r.state[0:time])
    merged_infomap_shot=merge_map(robotList)

    return robot_path_shot, merged_infomap_shot





def merge_map(robotList):
    """

    Args:
        robotList:

    Returns: merged_infomap 0: free space, 1: explored space, -1: obstacle

    """
    merged_map = np.zeros((y_n, x_n))
    data_3d = np.ndarray(shape=(y_n, x_n, 3), dtype=int)
    # define the color map
    color_map = {1: np.array([255, 255,153]),  # 1 is visited area filled with red
                 0: np.array([0, 102, 204]), # 0 is free space filled with blue color
                -1:np.array([192,192,192])}  # -1 is obstacle filled with gray color

    # define a 3D matrix to store the color value
    data_3d = np.ndarray(shape=(y_n, x_n, 3), dtype=int)

    # plot the grid_map with color
    for i in range(y_n):
        for j in range(x_n):
            sum_obs=0
            sum_explored=0
            for r in robotList:
                sum_obs+=r.tarobsmap[i,j]
                sum_explored+=r.infomap[i,j]
            if sum_obs<0:
                color=-1
            elif sum_explored>0:
                color=1
            else:
                color=0
            data_3d[i][j] = color_map[color]

    return data_3d




def visualize_motion(robot_list, mymap,num_time):
    """

    Args:
        robot_list: a list of robot objects
        mymap: environment map
        num_time: simulation time

    Returns:

    """
    fig = plt.figure('robot motion animation ', figsize=(6, 6))
    ax=plt.subplot(111, aspect = 'equal')
    line,=ax.plot([],[],'ro')
    plt.xlim(-10,x_bound+10)
    plt.ylim(-10,y_bound+10)
    for obstacle in mymap.obstacles:
        plt.plot(obstacle[0], obstacle[1])
    plt.title("robot motion animation")
    plt.xlabel("x coordinate")
    plt.ylabel("y coordinate")

    def init():
        line.set_data([],[])
        return line,

    def animate(time):

        X=[robot.state[time,0] for robot in robot_list]
        Y=[robot.state[time,1] for robot in robot_list]

        line.set_data(X,Y)

        return line,

    anim=animation.FuncAnimation(fig,
                                 animate,
                                 frames=num_time,
                                 init_func=init,
                                 blit=True,
                                 interval=100)

    # save video
    write_video = animation.FFMpegWriter(fps=10)  # fps is (frames per second)
    anim.save('../image/path_many_circles.mp4', writer=write_video)
    # plt.show()

    return anim

# plot and compare coverage percent curve

def compare_coverage_percent(file_list):
    figure = plt.figure('Coverage percent', figsize=(5, 5))
    file = file_list[0]
    c_percent = load_variavle(file)
    time = len(c_percent)
    time_steps = np.linspace(0, step_size * time, time)
    l1 = plt.plot(time_steps, c_percent, label='rc=2', color='k')

    file=file_list[1]
    c_percent=load_variavle(file)
    time=len(c_percent)
    time_steps = np.linspace(0, step_size * time, time )
    l2=plt.plot(time_steps, c_percent,label='rc=5', color='r')

    file = file_list[2]
    c_percent = load_variavle(file)
    time = len(c_percent)
    time_steps = np.linspace(0, step_size * time, time)
    l3 = plt.plot(time_steps, c_percent,label='rc=10',color='b')

    plt.legend()
    plt.xlabel("Simulation time")
    plt.ylabel("Coverage percent")
    plt.title("Area Coverage Percent")
    plt.grid(linewidth=0.5,color='lightgray')
    plt.savefig('../image/compare_coverage_percent_rc.png', dpi=200)

if __name__=="__main__":
    file_list=['../data/coverage_rc2','../data/coverage_rc5','../data/coverage_rc10']
    compare_coverage_percent(file_list)






