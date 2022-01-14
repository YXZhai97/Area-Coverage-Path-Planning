
import tikzplotlib
import matplotlib.pyplot as plt
import numpy as np
from global_value import *
from robot import show_merge_infomap, show_infomap


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
    plt.savefig('../image/target_34.png')


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
    plt.savefig('../image/coverage_percent34.png')


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
        "The robot path with iteration steps: %i " % time + ",time step: %1.2f s " % step_size + ",robot rs %1.2f m" % rs)

    # save png file and tex file
    plt.savefig('../image/path_4_rs.png')
    tikzplotlib.save("../tex_files/path34.tex")


def plot_robot_infomap(robotList, mymap):

    figure=plt.figure('robot information map ', figsize=(10, 10))
    subfig1 = figure.add_subplot(221)
    show_infomap(robotList[0], mymap)
    subfig2 = figure.add_subplot(222)
    show_infomap(robotList[1], mymap)
    subfig3=figure.add_subplot(223)
    show_infomap(robotList[2], mymap)
    subfig4 = figure.add_subplot(224)
    show_merge_infomap(robotList)
    plt.savefig('../image/infomap34.png')


