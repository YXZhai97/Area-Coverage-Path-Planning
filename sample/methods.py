import global_value as gv
import math
import numpy as np
import matplotlib.pyplot as plt
import robot
"""
define all the methods needed in one class
"""

def sigma_norm(z):
    '''
    Args:
        z: a array [x,y]
    Returns: a sigma norm number of the array
    '''
    z_sigma = (1 / gv.epsilon) * (math.sqrt(1 + gv.epsilon * (np.linalg.norm(z) ** 2)) - 1)
    return z_sigma


def adjacency_alpha(robot_list):
    '''
    Args:
        robot_list: a list containing robot objects

    Returns:
        a n by n adjacency matrix
    '''

    n = len(robot_list)
    A = np.zeros((n, n))
    rc=robot_list[0].rc
    for i in range(n):
        q_i = robot_list[i].initial_state[:2]
        for j in range(n):
            q_j=robot_list[j].initial_state[:2]
            if i != j:
                A[i, j] = bump_alpha(sigma_norm(q_j-q_i)/sigma_norm(rc))
            else:
                A[i,j]=0
    return A


def adjacency_beta():
    #todo
    pass


def bump_alpha(z):
    '''

    Args:
        z: a scalar value

    Returns: r, a scalar between 0 and 1

    '''
    r=0
    if z>=0 and z<gv.h_alpha:
        r=1
    elif z>=gv.h_alpha and z<1:
        r=0.5*(1+math.cos(math.pi*(z-gv.h_alpha)/(1-gv.h_alpha)))
    else:
        r=0
    return r


def bump_beta(z):
    '''

    Args:
        z: a scalar value

    Returns: r, a scalar value between 0 and 1

    '''
    r = 0
    if z >= 0 and z < gv.h_beta:
        r = 1
    elif z >= gv.h_beta and z < 1:
        r = 0.5 * (1 + math.cos(math.pi * (z - gv.h_beta) / (1 - gv.h_beta)))
    else:
        r = 0
    return r

def phi():
    pass



if __name__=="__main__":
    # test bump_alpha function
    z=np.linspace(-1,2,90)
    r=[]
    for zz in z:
        r.append(bump_alpha(zz))
    plt.plot(z,r)
    plt.title('The alpha bump function')
    plt.xlabel('z')
    plt.ylabel('bump factor')
    # plt.savefig('../image/bump_alpha.png')
    plt.show()

    # test the adjacency_alpha function
    robot.define_robot(6)
    for robot in gv.robotList:
        robot.random_init_state()
    print(adjacency_alpha(gv.robotList))
