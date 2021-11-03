"""
This file contains class and methods for plot the robot path
and animation of the robot behaviour

"""
from matplotlib import pyplot as plt
from matplotlib import animation
import numpy as np
import global_value as gv


def visualize(robot_list):


    fig = plt.figure()
    ax=plt.subplot(111, aspect = 'equal')
    line,=ax.plot([],[],'ro')
    plt.xlim(-1000,1000)
    plt.ylim(-1000,1000)

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
                                 frames=gv.Iteration,
                                 init_func=init,
                                 blit=True,
                                 interval=80)
    plt.show()

    return anim











