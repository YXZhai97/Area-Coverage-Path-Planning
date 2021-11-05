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
        q_i = np.array(robot_list[i].state[:2])
        for j in range(n):
            q_j=np.array(robot_list[j].state[:2])
            if i != j:
                A[i, j] = bump_alpha(sigma_norm(q_j-q_i)/sigma_norm(rc))
            else:
                A[i,j]=0
    return A


def adjacency_beta(beta_neighbour,q_i):
    n=len(beta_neighbour)
    if n<1:
        return 0
    B=np.zeros(n)
    k=0
    for beta in beta_neighbour:
        q_beta=beta[:2]
        B[k]=bump_beta(sigma_norm(q_beta-q_i)/sigma_norm(gv.d_beta))
        k+=1

    return B





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

def phi_alpha(z):
    '''

    Args:
        z: a scalar value

    Returns: a scalar

    '''
    d=sigma_norm(gv.d_alpha)
    y=bump_alpha(z/d)*(sigma_1(z-d)-1)
    return y


def phi_beta(z):
    '''

    Args:
        z: scalar

    Returns: y a scalar

    '''

    d = sigma_norm(gv.d_beta)
    y = bump_alpha(z / d) * (sigma_1(z - d) - 1)
    return y





def sigma_1(z):

    y=z/math.sqrt(1+(np.linalg.norm(z)**2))
    return y

def norm_direction(q_i,q_j):
    '''

    Args:
        q_i: a np state vector of robot i
        q_j: a np state vector of robot j

    Returns: norm vector

    '''
    n=np.zeros((1,2))
    n=(q_j-q_i)/math.sqrt(1+gv.epsilon*(np.linalg.norm(q_j-q_i)**2))
    return n





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

    # test the norm_direction() function
    q_i=np.array([100,56])
    q_j = np.array([100,5])
    print(norm_direction(q_i,q_j))

    # test sigma_norm
    print(sigma_norm([0,-51]))
    print(phi_alpha(sigma_norm([0,-51])))



