"""
Define alpha agent
The property of the alpha agent is defined
The method of the alpha agent is defined

"""
from typing import List

import matplotlib.pyplot as plt
import numpy as np
import math
import global_value as gv
import env_map as env
import methods as m

class Robot:
    number_of_robot = 0

    def __init__(self):
        self.id = self.number_of_robot
        self.rs = 10
        self.rc = 60
        self.dimension = 2
        self.map = []
        # initial_state=[x,y,v_x,v_y]
        self.initial_state = np.zeros((2 * self.dimension))
        self.state = np.zeros((1,4))
        self.neighbour = []
        self.initial_target=np.zeros((self.dimension))
        self.target=[]
        self.v_target=[]
        Robot.add_agent()  # call the class method when initialize the object

    @classmethod
    def add_agent(cls):
        cls.number_of_robot += 1

    def random_init_state(self):
        """

        Returns: set self.initial_state a random value [x,y] inside boundary

        """
        # Define initial velocity v_x, v_y
        self.initial_state[2]=np.random.uniform(0,1)*gv.v_bound
        self.initial_state[3]=np.random.uniform(0,1)*gv.v_bound
        # i is the current robot index
        i=self.id
        flag=0

        # Define initial state x, y
        while flag<=i:
            # sample x_i y_i from uniform distribution
            x_i = np.random.uniform(0, 1) * gv.x_bound
            y_i = np.random.uniform(0, 1) * gv.y_bound

            # compare x_i y_i to previously generated initial sate
            #  check the distance between them
            for j in range(i):
                x_j=gv.robotList[j].initial_state[0]
                y_j=gv.robotList[j].initial_state[1]
                distance=np.linalg.norm([x_i-x_j,y_i-y_j])
                # not consider initial state inside the obstacle
                # Todo consider the obstacle
                if distance<gv.d_alpha:
                    flag=0
                    break
                else:
                    flag+=1
            # if the distance between all initial state fulfill the condition
            if flag==i:
                gv.robotList[i].initial_state[0] = x_i
                gv.robotList[i].initial_state[1] = y_i
                break

    def random_init_target(self):
        """
        Returns: set self.initial_target a random value
        outside the sensing range and inside a boundary 6*sensing range
        the target should be integer value
        """
        flag=1
        while flag:
            center_x=self.initial_state[0]
            center_y=self.initial_state[1]
            r=np.random.uniform(self.rs,6*self.rs)
            theta=np.random.random()*2*math.pi
            x=center_x+r*math.cos(theta)
            y=center_y+r*math.sin(theta)
            if x>0 and y>0:
                flag=0
        self.initial_target[0]=round(x)
        self.initial_target[1]=round(y)


    def update_state(self):
        pass



    def get_neighbour(self):
        '''

        Returns: a list containing the robot's neighbour robots

        '''
        # get the id of the robot
        i=self.id
        # the current state of the robot
        q_i=np.array(self.state[:2])

        # find the neighbour in the robotList
        for r in gv.robotList:
            j=r.id
            q_j=np.array(r.state[:2])
            # check the distance within communication range
            if i!=j and np.linalg.norm(q_j-q_i)<self.rc:
                self.neighbour.append(r)
        return self.neighbour


    def control_input(self):
        # control input is two dimensional vector
        u = np.zeros((1, self.dimension))
        u_alpha = np.zeros((1, self.dimension))
        u_beta = np.zeros((1, self.dimension))
        u_gamma = np.zeros((1, self.dimension))
        i=self.id
        # position of the robot i
        q_i = np.array(self.state[:2])
        # velocity of robot i
        v_i = np.array((self.state[2:]))
        A = m.adjacency_alpha(gv.robotList)
        print(A)
        neighbour = self.get_neighbour()

        for robot_j in neighbour:
            j = robot_j.id
            a_ij = A[i,j]
            # position and velocity of robot j
            q_j = np.array(robot_j.state[:2])
            v_j = np.array(robot_j.state[2:])
            u_alpha_j_1=gv.c1_alpha*m.phi_alpha(m.sigma_norm(q_j-q_i))*m.norm_direction(q_i,q_j)
            u_alpha_j_2=gv.c2_alpha*a_ij*(v_j-v_i)
            u_alpha+=u_alpha_j_1
        return u_alpha




    def benefit_value(self):
        pass





class BetaAgent():
    def __init__(self):
        pass




def define_robot(number):
    """
    Args:
        number: The number of agents in the map

    Returns: Append robotList with new instances
    """
    for i in range(number):
        gv.robotList.append(Robot())



if __name__=="__main__":

    # test the random_init_state function
    # first define 4 robots
    define_robot(6)
    print(gv.robotList[0].initial_state)
    # plot the initial state point on map with id number as label
    for robot in gv.robotList:
        robot.random_init_state()
        robot.random_init_target()
        plt.scatter(robot.initial_state[0],robot.initial_state[1])
        plt.scatter(robot.initial_target[0], robot.initial_target[1],marker='*',color='black')
        plt.annotate(robot.id,(robot.initial_state[0],robot.initial_state[1]))
        plt.annotate(robot.id, (robot.initial_target[0], robot.initial_target[1]))
        print('initial state:',robot.initial_state)
        print('initial target:',robot.initial_target)
    plt.title('The initial state of robots')
    plt.xlabel('x-coordinate')
    plt.ylabel('y-coordinate')
    # save image to the image folder
    # plt.savefig('../image/initial_state.png')
    plt.show()

    # test get_neighbour()
    gv.robotList[0].state=[24,56,0,0]
    gv.robotList[1].state = [2, 56, 0, 0]
    gv.robotList[2].state = [90, 56, 0, 0]
    gv.robotList[3].state = [100, 60, 0, 0]
    gv.robotList[4].state = [53, 56, 0, 0]
    gv.robotList[5].state = [54, 90, 0, 0]
    print(gv.robotList[2].get_neighbour())
    print(gv.robotList[5])
    print(gv.robotList[2].neighbour)
    print(m.adjacency_alpha(gv.robotList))

    # test control input
    print(gv.robotList[2].control_input())

    a=np.array([1,2])
    b=np.array([3,4])

