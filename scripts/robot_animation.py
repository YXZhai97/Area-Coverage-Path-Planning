"""
This file contains class and methods for plot the robot path
and animation of the robot behaviour

"""
from matplotlib import pyplot as plt
from matplotlib import animation
import global_value as gv


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
    plt.xlim(-10,gv.x_bound+10)
    plt.ylim(-10,gv.y_bound+10)
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
    anim.save('../image/robot_path_animation32.mp4', writer=write_video)
    # plt.show()

    return anim













