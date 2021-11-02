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
import collections


class Robot:
    number_of_robot = 0

    def __init__(self):
        self.id = self.number_of_robot
        self.rs = 10
        self.rc = 60
        self.dimension = 2

        # information map
        self.infomap = np.zeros((gv.x_n, gv.y_n))
        self.tarobsmap = np.zeros((gv.x_n, gv.y_n))

        # initial_state=[x,y,v_x,v_y]
        self.initial_state = np.zeros(2 * self.dimension)
        self.state = np.zeros((gv.Iteration, 2 * self.dimension))

        # alpha neighbour: a list containing neighbour robots
        self.neighbour = []

        # beta neighbour [[],[],[]] a list of list
        # beta neighbour [[x1,y1,vx1,vy1],[x2,y2,vx2,vy2]]
        self.beta_neighbour = []

        # target position [x,y], no velocity target
        self.initial_target = np.zeros(self.dimension)
        self.target = np.zeros((gv.Iteration, self.dimension))

        # benefit matrix row and column equal env_map dimension
        self.benefit_matrix = np.zeros((gv.x_n, gv.y_n))
        Robot.add_agent()  # call the class method when initialize the object

    # class method to add number of robot
    @classmethod
    def add_agent(cls):
        cls.number_of_robot += 1

    def random_init_state(self):
        """

        Returns: set self.initial_state a random value [x,y,0,0] inside boundary

        """
        # Define initial velocity v_x, v_y
        self.initial_state[2] = np.random.uniform(0, 1) * gv.v_bound
        self.initial_state[3] = np.random.uniform(0, 1) * gv.v_bound
        # i is the current robot index
        i = self.id
        flag = 0

        # Define initial state x, y
        while flag <= i:
            # sample x_i y_i from uniform distribution
            # todo fix the random initial later
            x_i = np.random.uniform(0, 1) * gv.x_bound
            y_i = np.random.uniform(0, 1) * gv.y_bound

            # compare x_i y_i to previously generated initial sate
            #  check the distance between them
            for j in range(i):
                x_j = gv.robotList[j].initial_state[0]
                y_j = gv.robotList[j].initial_state[1]
                distance = np.linalg.norm([x_i - x_j, y_i - y_j])
                # not consider initial state inside the obstacle
                # Todo consider the obstacle
                if distance < gv.d_alpha:
                    flag = 0
                    break
                else:
                    flag += 1
            # if the distance between all initial state fulfill the condition
            if flag == i:
                gv.robotList[i].initial_state[0] = x_i
                gv.robotList[i].initial_state[1] = y_i

                # the initial state is [x,y,0,0] with initial velocity 0
                gv.robotList[i].state[0, :] = [x_i, y_i, 0, 0]
                break

    def random_init_target(self):
        """
        Returns: set self.initial_target a random value
        outside the sensing range and inside a boundary 6*sensing range
        the target should be integer value
        """
        flag = 1
        while flag:
            center_x = self.initial_state[0]
            center_y = self.initial_state[1]
            r = np.random.uniform(self.rs, 6 * self.rs)
            theta = np.random.random() * 2 * math.pi
            x = center_x + r * math.cos(theta)
            y = center_y + r * math.sin(theta)
            if x > 0 and y > 0:
                flag = 0
        # target can also be float, a round may not be necessary
        # todo delete the round()
        self.initial_target[0] = round(x)
        self.initial_target[1] = round(y)
        # 0 row of target matrix is the random initial target
        self.target[0] = self.initial_target

    def update_state(self, time):
        # get the current state
        cur_position = self.state[time, :2]
        cur_v = self.state[time, 2:]

        # calculate the deviation
        d_v = self.control_input(time) * gv.step_size
        d_position = self.state[time, 2:] * gv.step_size

        # add new state vector
        self.state[time + 1, :2] = cur_position + d_position
        self.state[time + 1, 2:] = cur_v + d_v

    def get_neighbour(self, time):
        '''
        Returns: a list containing the robot's neighbour robots
        '''
        # get the id of the robot
        i = self.id
        # the current state of the robot
        q_i = np.array(self.state)[time, :2]

        # find the neighbour in the robotList
        for r in gv.robotList:
            j = r.id
            q_j = np.array(r.state)[time, :2]
            # check the distance within communication range
            if i != j and np.linalg.norm(q_j - q_i) < self.rc:
                self.neighbour.append(r)
        return self.neighbour

    def control_input(self, time):
        # control input is two dimensional vector
        u = np.zeros((1, self.dimension))
        # control input has 3 parts
        u_alpha = np.zeros((1, self.dimension))
        u_beta = np.zeros((1, self.dimension))
        u_gamma = np.zeros((1, self.dimension))
        i = self.id
        # position of the robot i at time t
        q_i = np.array(self.state[time, :2])
        # velocity of robot i
        v_i = np.array(self.state[time, 2:])
        A = m.adjacency_alpha(gv.robotList)
        B = m.adjacency_beta(self.beta_neighbour, q_i)
        neighbour = self.get_neighbour(time)

        # calculate the influence of alpha neighbour
        for robot_j in neighbour:
            j = robot_j.id
            a_ij = A[i, j]
            # position and velocity of robot j
            q_j = np.array(robot_j.state)[time, :2]
            v_j = np.array(robot_j.state)[time, 2:]
            u_alpha_j_1 = gv.c1_alpha * m.phi_alpha(m.sigma_norm(q_j - q_i)) * m.norm_direction(q_i, q_j)
            u_alpha_j_2 = gv.c2_alpha * a_ij * (v_j - v_i)
            u_alpha += u_alpha_j_1 + u_alpha_j_2

        # calculate the influence of beta_agent
        # first get the neighbour
        self.get_beta(time)
        k = 0
        for beta in self.beta_neighbour:
            q_beta = np.array(beta[:2])
            v_beta = np.array(beta[2:])
            # todo ajacency matrix B
            b_ik = B[k]  # i and k
            u_beta_k_1 = gv.c1_beta * m.phi_beta(m.sigma_norm(q_beta - q_i)) * m.norm_direction(q_beta, q_i)
            u_beta_k_2 = gv.c2_beta * b_ij * (v_beta - v_i)
            u_beta += u_beta_k_1 + u_beta_k_2
            k += 1

        # calculate the influence of gamma_agent
        # target is Iteration*2 dimensional matrix
        q_target = self.target[time]
        u_gamma = -gv.c1_gamma * (q_i - q_target) - gv.c2_gamma * v_i

        # limit gamma attraction
        norm_u_gamma = np.linalg.norm(u_gamma)
        if norm_u_gamma > 150:
            u_gamma = 150 * u_gamma / norm_u_gamma

        # merge u_alpha, u_beta, u_gamma
        u = u_alpha + u_beta + u_gamma

        # limit the acceleration
        norm_u = np.linalg.norm(u)
        if norm_u > 100:
            u = 100 * u / norm_u

        return u

    def benefit_value(self, time):
        lamda_matrix = np.zeros((gv.x_n, gv.y_n))
        for i in range(gv.x_n):
            for j in range(gv.y_n):
                x_center = i * gv.grid_length + 0.5 * gv.grid_length
                y_center = j * gv.grid_length + 0.5 * gv.grid_length
                center = np.array([x_center, y_center])

                # if the current cell is occupied by an obstacle
                if self.tarobsmap[i, j] < 0:
                    self.benefit_matrix[i, j] = 0
                else:
                    element1 = -gv.k1 * np.linalg.norm(self.state[time, :2] - center)
                    element2 = -gv.k2 * np.linalg.norm(self.target - center)
                    lamda_matrix[i, j] = math.exp(element1 + element2)
                    self.benefit_matrix[i, j] = (1 - self.infomap[i, j]) * (
                            gv.rohgamma + (1 - gv.rohgamma) * lamda_matrix[i, j])
        return self.benefit_matrix

    # get the maximum value in benefit_matrix
    # todo review code
    def update_target(self, time):

        '''
        Args:
            time: get the time step
        Returns:
            return the maximum value from the benefit_matrix
        '''

        # get the current state and information map
        # robot state
        q_i = self.state[time, :2]
        infomap = self.infomap
        tarobsmap = self.tarobsmap
        cur_target = self.target[time]
        new_target = cur_target

        # recalculation criteria
        a, b, c = 0, 0, 0

        # first criteria
        if m.sigma_norm(q_i - cur_target) < m.sigma_norm(self.rs) or time == 1:
            a = 1

        # calculate the row and col of the last target
        row = int((cur_target[0] - gv.grid_length / 2) / gv.grid_length)
        col = int((cur_target[1] - gv.grid_length / 2) / gv.grid_length)

        # second criteria
        # todo debug row and col out of range
        if infomap[row, col] != 0 or tarobsmap[row, col] != 0:
            b = 1

        # third criteria
        neighbour = self.get_neighbour(time)
        for robot_j in neighbour:
            q_j = robot_j.state[time, :2]
            cur_target_j = robot_j.target[time]
            if m.sigma_norm(cur_target - cur_target_j) < self.rs and m.sigma_norm(q_j - cur_target_j) < m.sigma_norm(
                    q_i - cur_target):
                c = 1
                break

        def get_center(row, col):
            x_center = gv.grid_length / 2 + row * gv.grid_length
            y_center = gv.grid_length / 2 + col * gv.grid_length
            cell_center = np.array([x_center, y_center])
            return cell_center

        # if recalculation condition fulfilled
        if a == 1 or b == 1 or c == 1:

            # create a priority queue
            # the queue stores [x,y,benefit_value]
            b_value_que = np.zeros((1, 3))

            b_value = self.benefit_value(time)

            n_neighbour = len(neighbour)
            # if the robot has neighbour robot

            for x in range(gv.x_n):
                for y in range(gv.y_n):
                    center = get_center(x, y)
                    if n_neighbour > 0:
                        for robot_j in neighbour:
                            center = get_center(x, y)
                            q_j = robot_j.state[time, :2]
                            if m.sigma_norm(center[:2] - q_j) >= m.sigma_norm(center[:2] - q_i) > self.rs:
                                center = np.append(center, b_value[x, y])
                                # set the last row of Xi_que [x,y,benefit_value]
                                # append the queue with [0,0,0]
                                b_value_que[-1] = center
                                newrow = np.zeros(3)
                                b_value_que = np.append(b_value_que, [newrow], axis=0)

                    # if the robot has no neighbour at all
                    elif m.sigma_norm(center - q_i) > self.rs:
                        center = np.append(center, b_value[x, y])
                        b_value_que[-1] = center
                        newrow = np.zeros(3)
                        b_value_que = np.append(b_value_que, [newrow], axis=0)

            # get the max value from the b_value_que
            max_b_value = b_value_que[0]
            for i in range(1, len(b_value_que)):
                if b_value_que[i, 2] > max_b_value[2]:
                    max_b_value = b_value_que[i]

            if max_b_value[2] > 0:
                new_target = max_b_value[:2]
            else:
                new_target = cur_target

        # when the first three criteria does not work
        else:
            for robot_j in neighbour:
                q_j = robot_j.state[time, :2]
                cur_target_j = robot_j.target[time]
                if (m.sigma_norm(q_i - cur_target_j) < m.sigma_norm(q_i - cur_target) and
                        m.sigma_norm(q_i - cur_target_j) < m.sigma_norm(q_j - cur_target_j)):
                    # exchange target i and j
                    new_target = cur_target_j
                    robot_j.target[time + 1] = cur_target
                else:
                    new_target = cur_target

        self.target[time + 1] = new_target
        return new_target

    def update_info_map(self, time):

        '''
        Args:
            time: give the time step as input
        Returns:
            no return, but the self.infomap and self.tarobsmap is updated
        '''

        # get the current state
        q = np.array(self.state[time, :2])

        # update the infomap and tarobsmap inside rs
        for x in range(gv.x_n):
            for y in range(gv.y_n):
                # calculate the cell center position
                x_center = gv.grid_length / 2 + x * gv.grid_length
                y_center = gv.grid_length / 2 + y * gv.grid_length
                cell_center = np.array([x_center, y_center])

                # calculate the distance between cell center and robot position
                distance = np.linalg.norm(q - cell_center)

                # mark the visited area in info_map
                if distance < self.rs:
                    self.infomap[y, x] = 1

                if distance < self.rs and gv.env_map[y, x] == 1:
                    self.tarobsmap[y, x] = -1

        # consider communication and map info exchange between robots with in rc
        neighbour = self.get_neighbour(time)
        for robot_j in neighbour:
            for x in range(gv.x_n):
                for y in range(gv.y_n):
                    if robot_j.infomap[y, x] > self.infomap[y, x]:
                        self.infomap[y, x] = robot_j.infomap[y, x]

                    if robot_j.tarobsmap[y, x] != 0 and self.tarobsmap[y, x] == 0:
                        self.tarobsmap[y, x] = robot_j.tarobsmap[y, x]

                    if robot_j.tarobsmap[y, x] == 0 and self.tarobsmap[y, x] != 0:
                        robot_j.tarobsmap[y, x] = self.tarobsmap[y, x]

    # def get_beta_agent(self, time):
    #     # robot current sate
    #     q = np.array(self.state)[time, :2]
    #
    #     # find the closest point on obstacle
    #     close_distance = self.rs
    #     for x in range(gv.x_n):
    #         for y in range(gv.y_n):
    #             if self.tarobsmap[x, y] == -1:
    #                 x_center = gv.grid_length / 2 + x * gv.grid_length
    #                 y_center = gv.grid_length / 2 + y * gv.grid_length
    #                 cell_center = np.array([x_center, y_center])
    #                 distance = np.linalg.norm(q - cell_center)
    #                 # update the closest point
    #                 # set the closest point as beta neighbour
    #                 # how to tell the point on different obstacle
    #                 # todo
    #                 if distance < close_distance:
    #                     close_distance = distance
    #                     beta_index = [x, y]
    #
    #     # get the position of the beta agent
    #     beta_x = gv.grid_length / 2 + beta_index[0] * gv.grid_length
    #     beta_y = gv.grid_length / 2 + beta_index[1] * gv.grid_length
    #
    #     # get the velocity projection of the beta agent
    #     beta_vx = 0
    #     beta_vy = 0
    #     beta_state = [beta_x, beta_y, beta_vx, beta_vy]
    #     # add the beta_state to beta_neighbour list
    #     self.beta_neighbour.append(beta_state)

    # get the nearest beta point on k obstacle around the robot
    def get_beta(self, time):
        grid = self.tarobsmap
        rows, cols = len(grid), len(grid[0])
        visit = set()
        beta_neighbour = []
        grid_length = gv.grid_length
        rs = self.rs

        # the position and velocity of the robot
        position = np.array(self.state[time, :2])
        velocity = np.array(self.state[time, 2:])

        def get_center(row, col):
            x_center = grid_length / 2 + row * grid_length
            y_center = grid_length / 2 + col * grid_length
            cell_center = np.array([x_center, y_center])
            return cell_center

        def bfs(r, c):
            q = collections.deque()
            visit.add((r, c))
            q.append((r, c))
            min_distance = rs

            while q:
                row, col = q.popleft()
                center = get_center(row, col)
                distance = np.linalg.norm(position - center)
                if distance < min_distance:
                    min_distance = distance
                    nearest_neighbour = center

                directions = [[1, 0], [-1, 0], [0, 1], [0, -1]]
                for dr, dc in directions:
                    r, c = row + dr, col + dc
                    if (r in range(rows) and
                            c in range(cols) and
                            grid[r][c] == -1 and
                            (r, c) not in visit):
                        q.append((r, c))
                        visit.add((r, c))
            return nearest_neighbour

        def v_projection(p, p_beta, v):
            '''
            Args:
                p: current robot position
                p_beta: beta agent position
                v: current robot velocity
            Returns:
                robot velocity projection
            '''
            p = np.array(p)
            p_beta = np.array((p_beta))
            # identity matrix with dimension 2
            I = np.identity(self.dimension)
            # a_k is unit normal vector
            a_k = (p - p_beta) / np.linalg.norm(p - p_beta)
            # projection matrix
            proj_matrix = I - np.outer(a_k, a_k.T)
            # project velocity
            v_proj = np.dot(v, proj_matrix)
            return v_proj

        for r in range(rows):
            for c in range(cols):
                if grid[r][c] == -1 and (r, c) not in visit:
                    beta_neighbour.append(bfs(r, c))

        for p_beta in beta_neighbour:
            v_proj = v_projection(position, p_beta, velocity)
            # add the velocity to the vector
            p_beta.append(v_proj[0])
            p_beta.append(v_proj[1])

        self.beta_neighbour = beta_neighbour
        return beta_neighbour


def define_robot(number):
    """
    Args:
        number: The number of agents in the map

    Returns:
        Append robotList with new instances
    """
    for i in range(number):
        gv.robotList.append(Robot())

    return gv.robotList


# show the robot initial state and target in plot
def show_robot(robotList):
    for robot in robotList:
        plt.scatter(robot.initial_state[0], robot.initial_state[1])
        plt.scatter(robot.initial_target[0], robot.initial_target[1], marker='*', color='black')
        plt.annotate(robot.id, (robot.initial_state[0], robot.initial_state[1]))
        plt.annotate(robot.id, (robot.initial_target[0], robot.initial_target[1]))
        print('initial state:', robot.initial_state)
        print('initial target:', robot.initial_target)

    plt.title('The initial state of robots')
    # add x-label
    plt.xlabel('x-coordinate')
    # add y label
    plt.ylabel('y-coordinate')
    # save image to the image folder
    # plt.savefig('../image/initial_state.png')
    plt.show()


if __name__ == "__main__":

    # test the random_init_state function
    # first define 4 robots
    define_robot(6)
    print(gv.robotList[0].initial_state)
    # plot the initial state point on map with id number as label
    for robot in gv.robotList:
        robot.random_init_state()
        robot.random_init_target()
        plt.scatter(robot.initial_state[0], robot.initial_state[1])
        plt.scatter(robot.initial_target[0], robot.initial_target[1], marker='*', color='black')
        plt.annotate(robot.id, (robot.initial_state[0], robot.initial_state[1]))
        plt.annotate(robot.id, (robot.initial_target[0], robot.initial_target[1]))
        print('initial state:', robot.initial_state)
        print('initial target:', robot.initial_target)
    plt.title('The initial state of robots')
    # add x-label
    plt.xlabel('x-coordinate')
    # add y label
    plt.ylabel('y-coordinate')
    # save image to the image folder
    # plt.savefig('../image/initial_state.png')
    plt.show()

    # test get_neighbour()
    gv.robotList[0].state[1] = [24, 56, 0, 0]
    gv.robotList[1].state[1] = [2, 56, 0, 0]
    gv.robotList[2].state[1] = [90, 56, 0, 0]
    gv.robotList[3].state[1] = [100, 60, 0, 0]
    gv.robotList[4].state[1] = [53, 56, 0, 0]
    gv.robotList[5].state[1] = [54, 90, 0, 0]
    time = 1
    print(gv.robotList[2].get_neighbour(time))
    print(gv.robotList[5])
    print(gv.robotList[2].neighbour)
    print(m.adjacency_alpha(gv.robotList))

    # test control input
    print(gv.robotList[2].control_input(1))

    a = np.array([1, 2])
    b = np.array([3, 4])

    # test benefit_value function
    benefit_value = gv.robotList[1].benefit_value()
    print(benefit_value)
    print(len(benefit_value))
    print(len(benefit_value[0]))

    # test update infomap
    mymap = env.EnvMap(300, 300, 1)
    mymap.add_circle(90, 60, 45)
    gv.env_map = mymap.grid_map
    gv.robotList[1].update_info_map(1)
    print(gv.robotList[1].infomap)
