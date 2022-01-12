from robot_animation import *
import tikzplotlib
import matplotlib.pyplot as plt
import numpy as np
from global_value import *


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
        "The robot path with simulation time %1.2f " % time*step_size + ",time step %1.2f s " % step_size + ",robot number %i" % robot_number)
    plt.savefig('../image/target_4_rs.png')


def plot_coverage_percent(c_percent, time):
    '''
    This function plots the coverage percent
    Args:
        c_percent: a lsit of coverage_percent number
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
    plt.savefig('../image/coverage_percent.png')


def plot_robot_path(robotList, mymap, time):

    """
    This function plots the robot path and obstacles
    Args:
        robotList: a list of robot object
        mymap: the grid map
        time: the total simulation time

    Returns: save path image files

    """
    figure= plt.figure('robot path ', figsize=(6, 6))

    # plot the robot path
    for robot in robotList:
        plt.plot(robot.state[:time, 0], robot.state[:time, 1])

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
        "The robot path with simulation time %i " % time*step_size + ",time step %1.3f s " % step_size + ",robot number %i" % robot_number)

    # save png file and tex file
    plt.savefig('../image/path_4_rs.png')
    tikzplotlib.save("../tex_files/path_32.tex")


def plot_robot_animation(robotList, mymap, time):
    """

    Args:
        robotList:
        mymap:
        time:

    Returns:

    """
    anim = visualize(robotList, mymap, time)
    write_video = animation.FFMpegWriter(fps=10)  # fps is (frames per second)
    anim.save('../image/robot_path_animation_4_rs.mp4', writer=write_video)


def plot_robot_infomap():
    pass
