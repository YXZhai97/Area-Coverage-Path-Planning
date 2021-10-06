"""
Define alpha agent
The property of the alpha agent is defined
The method of the alpha agent is defined

"""
from typing import List
import numpy as np
import weakref
import global_value as gv
import env_map as env

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
        self.state = []
        self.neighbour = []
        Robot.add_agent()  # call the class method when initialize the object

    @classmethod
    def add_agent(cls):
        cls.number_of_robot += 1

    def random_init_state(self):
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







def define_robot(number):
    """

    Args:
        number: The number of agents in the map

    Returns: Append robotList with new instances

    """
    for i in range(number):
        gv.robotList.append(Robot())


if __name__=="__main__":

    define_robot(4)
    print(gv.robotList[0].initial_state)
    for robot in gv.robotList:
        robot.random_init_state()
        print(robot.initial_state)
    print(gv.robotList[0].initial_state)
    print(gv.robotList[1].id)
    robot5=Robot()
    print(robot5.id)

