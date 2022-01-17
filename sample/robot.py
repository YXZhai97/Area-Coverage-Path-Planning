"""
Define alpha agent
The property of the alpha agent is defined
The method of the alpha agent is defined

"""

import matplotlib.pyplot as plt
import numpy as np
import math
import global_value as gv
import env_map as env
import methods as m
import collections
from tangentbug import *
from floodFill import flood_fill

class Robot:
    number_of_robot = 0

    def __init__(self):
        # robot parameters
        self.id = self.number_of_robot
        self.rs = gv.rs
        self.rc = gv.rc
        # r_tan: tangent bug distance to the obstacle
        self.r_tan=gv.r_tan
        self.target_distance=gv.target_distance
        self.dimension = gv.dimension

        # robot motion mode
        # free space exploration 0
        # boundary following 1
        self.motion_mode=0
        # d_tan is the activation distance for tangent bug
        self.d_tan=self.rs/5
        # d_position_limit, the limit for updating the state of each step
        self.d_position_limit=1

        # information map
        self.infomap = np.zeros((gv.y_n, gv.x_n))
        self.tarobsmap = np.zeros((gv.y_n, gv.x_n))
        self.coverage_percent=np.zeros(gv.Iteration)
        self.step_len=0.4
        self.tangent_end_time=0
        self.tangent_start_time=0
        self.inside_tangent_planner=False
        self.tangent_targets=[]
        self.boundary_follow_finished=False
        self.followed_curve=[]

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
        self.benefit_matrix = np.zeros((gv.y_n, gv.x_n))
        Robot.add_agent()  # call the class method when initialize the object

    # class method to add number of robot
    @classmethod
    def add_agent(cls):
        cls.number_of_robot += 1


    def random_init_state(self, mymap):
        """

        Returns: set self.initial_state a random value [x,y,0,0] inside boundary

        """
        # get the obstacles from mymap
        obstacles=mymap.obstacles
        bounding_boxes=[] # store the min max value of the obstacles

        # get the x_min, x_max, y_min, y_max of the obstacles
        for obstacle in obstacles:
            x_min, y_min=min(obstacle[0]), min(obstacle[1])
            x_max, y_max=max(obstacle[0]), max(obstacle[1])
            bounding_boxes.append([x_min, x_max, y_min, y_max])

        # Define initial velocity v_x, v_y
        self.initial_state[2] = np.random.uniform(0.1, 0.9) * gv.v_bound
        self.initial_state[3] = np.random.uniform(0.1, 0.9) * gv.v_bound
        # i is the current robot index
        i = self.id
        flag_obs=0
        flag_distance = 0

        # Define initial state x, y
        while flag_distance <= i:
            # sample x_i y_i from uniform distribution
            # todo fix the random initial later
            while not flag_obs:
                inside=0
                x_i = np.random.uniform(0.1, 0.9) * gv.x_bound
                y_i = np.random.uniform(0.1, 0.9) * gv.y_bound
                for bound in bounding_boxes:
                    if bound[0]-self.r_tan<x_i<bound[1]+self.r_tan and bound[2]-self.r_tan<y_i<bound[3]+self.r_tan:
                        inside=1
                        break
                    else:
                        continue
                if inside:
                    flag_obs=0
                else:
                    flag_obs=1

            flag_obs=0
            # compare x_i y_i to previously generated initial sate
            #  check the distance between them
            for j in range(i):
                x_j = gv.robotList[j].initial_state[0]
                y_j = gv.robotList[j].initial_state[1]
                # check the distance with other agents
                distance = np.linalg.norm([x_i - x_j, y_i - y_j])
                if distance < gv.d_alpha:
                    flag_distance = 0
                    break
                else:
                    flag_distance += 1
            # if the distance between all initial state fulfill the condition
            if flag_distance == i:
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
            # todo the range of the initial state
            r = np.random.uniform(self.rs, 2* self.rs)
            theta = np.random.random() * 2 * math.pi
            x = center_x + r * math.cos(theta)
            y = center_y + r * math.sin(theta)
            if gv.x_bound*0.8 > x > 0.2*gv.x_bound and gv.y_bound*0.8 > y > gv.y_bound*0.2:
                flag = 0
        # target can also be float, a round may not be necessary
        # todo delete the round()
        self.initial_target[0] = round(x)
        self.initial_target[1] = round(y)
        # 0 row of target matrix is the random initial target
        self.target[0] = self.initial_target

    def set_init_state(self,x, y):
        """
        set the initial state and target
        Returns:no return, update the self.state

        """
        self.state[0,:]=[x, y, 0, 0]
        self.initial_state=[x, y, 0, 0]

    def set_init_target(self, x, y):
        """
        set the initial target manually
        Returns: no return, update the self.target

        """
        self.target[0]=[x,y]
        self.initial_target=[x,y]

    def update_state(self, time):
        # get the current state
        cur_position = self.state[time, :2]
        cur_v = self.state[time, 2:]

        # calculate the deviation
        d_v = self.control_input(time) * gv.step_size*gv.rate
        d_position = self.state[time, 2:] * gv.step_size*gv.rate

        # limit the position change at each time step
        norm_d_position = np.linalg.norm(d_position)
        if norm_d_position>self.d_position_limit:
            d_position=self.d_position_limit*d_position/norm_d_position


        # update the robot state for the next time step
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
        beta_neighbour=self.get_beta(time)
        neighbour=self.get_neighbour(time)
        # control input is two dimensional vector
        u = np.zeros((1, self.dimension))
        # control input has 3 parts
        u_alpha = np.zeros((1, self.dimension))
        u_beta = np.zeros((1, self.dimension))
        u_gamma = np.zeros((1, self.dimension))
        i = self.id
        # position of the robot i at time t
        q_i = self.state[time, :2]
        # velocity of robot i
        v_i = self.state[time, 2:]
        A = m.adjacency_alpha(gv.robotList)
        B = m.adjacency_beta(self.beta_neighbour, q_i)

        # print(B)
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

        # reset the alpha force to zero when robot out of boundary
        if q_i[0] > gv.x_bound or q_i[0]<0 or q_i[1]>gv.y_bound or q_i[1]<0:
            u_alpha=0

        # limit the alpha force
        norm_u_alpha = np.linalg.norm(u_alpha)
        if norm_u_alpha > 50:
            u_alpha = 50 * u_alpha / norm_u_alpha

        # print("u_alpha:", u_alpha)

        # calculate the influence of beta_agent
        # first get the neighbour
        # todo repulsive force too large, when robots meet, separate too quick

        # the influence of beta-agent is deleted
        # if len(beta_neighbour)>0:
        #     k = 0
        #     for beta in beta_neighbour:
        #         q_beta = np.array(beta)[:2]
        #         v_beta = np.array(beta)[2:]
        #         # todo adjacency matrix B
        #         b_ik = B[k]  # i and k
        #         # print(b_ik)
        #         u_beta_k_1 = gv.c1_beta * m.phi_beta(m.sigma_norm(q_beta - q_i)) * m.norm_direction(q_beta, q_i)
        #         # u_beta_k_2 = np.multiply((v_beta - v_i),gv.c2_beta * b_ik )
        #         u_beta_k_2 = (gv.c2_beta * b_ik) * (v_beta - v_i)
        #         u_beta += u_beta_k_1 + u_beta_k_2
        #         k += 1
        #
        # # if robot out of the boundary, the repulsion force is cancelled
        # if q_i[0] > gv.x_bound or q_i[0]<0 or q_i[1]>gv.y_bound or q_i[1]<0:
        #     u_beta=0

        # calculate the influence of gamma_agent
        # target is Iteration*2 dimensional matrix
        q_target = self.target[time]
        u_gamma = -gv.c1_gamma * (q_i - q_target) - gv.c2_gamma * v_i

        # limit gamma attraction
        norm_u_gamma = np.linalg.norm(u_gamma)
        if norm_u_gamma > 50:
            u_gamma = 50 * u_gamma / norm_u_gamma
        # print("u_gamma:", u_gamma)
        # merge u_alpha, u_beta, u_gamma
        u = u_alpha + u_beta + u_gamma
        # print(u)
        # limit the acceleration
        norm_u = np.linalg.norm(u)
        if norm_u > 40:
            u = 40 * u / norm_u

        # print(u)
        # print(norm_u)

        return u


    def benefit_value(self, time):


        lamda_matrix = np.zeros((gv.y_n, gv.x_n))
        for i in range(gv.x_n):
            for j in range(gv.y_n):
                x_center = i * gv.grid_length + 0.5 * gv.grid_length
                y_center = j * gv.grid_length + 0.5 * gv.grid_length
                center = np.array([x_center, y_center])

                # if the current cell is occupied by an obstacle
                if self.tarobsmap[j, i] < 0:
                    self.benefit_matrix[j, i] = 0
                else:
                    element1 = -gv.k1 * np.linalg.norm(self.state[time, :2] - center)
                    element2 = -gv.k2 * np.linalg.norm(self.target[time] - center)
                    lamda_matrix[j, i] = math.exp(element1 + element2)
                    self.benefit_matrix[j, i] = (1 - self.infomap[j, i]) * (
                            gv.rohgamma + (1 - gv.rohgamma) * lamda_matrix[j, i])

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
        col = int(cur_target[0] // gv.grid_length)  # x//grid_length is col
        row = int(cur_target[1] // gv.grid_length)  # y//grid_length is row

        # second criteria
        # todo debug row and col out of range
        if infomap[row, col] != 0 or tarobsmap[row, col] != 0:
            b = 1

        # third criteria
        neighbour = self.get_neighbour(time)
        for robot_j in neighbour:
            q_j = robot_j.state[time, :2]
            cur_target_j = robot_j.target[time]
            # todo debug
            if m.sigma_norm(cur_target - cur_target_j) < self.rs and m.sigma_norm(q_j - cur_target_j) < m.sigma_norm(
                    q_i - cur_target) and robot_j.motion_mode==0:
                c = 1
                break

        def get_center(x, y):
            x_center = gv.grid_length / 2 + x * gv.grid_length
            y_center = gv.grid_length / 2 + y * gv.grid_length
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
                            if m.sigma_norm(center[:2] - q_j) >= m.sigma_norm(center[:2] - q_i) > self.target_distance:
                                center = np.append(center, b_value[y, x])
                                # set the last row of Xi_que [x,y,benefit_value]
                                # append the queue with [0,0,0]
                                b_value_que[-1] = center
                                newrow = np.zeros(3)
                                b_value_que = np.append(b_value_que, [newrow], axis=0)

                    # if the robot has no neighbour at all
                    elif m.sigma_norm(center - q_i) > self.target_distance:
                        center = np.append(center, b_value[y, x])
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

    def check_inside_obs(self, time, mymap):

        cur_target=self.target[time]
        for circle in mymap.circle_bound:
            distance=sqrt((cur_target[0]-circle[0])**2+(cur_target[1]-circle[1])**2)
            if distance < circle[2]:
                return True

        for bound in mymap.polygon_bound:
            if bound[0]<cur_target[0]<bound[1] and bound[2]<cur_target[1]<bound[3]:
                return True

        return False






    def update_info_map(self, time, mymap):

        '''
        Args:
            time: give the time step as input
            mymap: a map object
        Returns:
            no return, but the self.infomap and self.tarobsmap is updated
        '''

        # get the current state
        q = np.array(self.state[time, :2])
        grid_map=mymap.grid_map
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

                # update the obstacles
                if distance < self.rs and grid_map[y, x] == 1:
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

        def update_percent(infomap,time):
            coverage_area=np.sum(self.infomap)
            area=gv.x_n*gv.y_n
            percent=coverage_area/area
            self.coverage_percent[time]=percent

        update_percent(self.infomap,time)


    def update_motion_mode(self, time, mymap, new_target):
        """
        Args:
            time: the time step
            mymap: the obstacle grid map
            new_target: the new_target from update target

        Returns: the motion_mode of the robot
        """
        cur_state=self.state[time, :2]
        obstacles=mymap.obstacles
        rs=self.rs
        # check the intersection
        # use the import tangent bug functions
        is_intersect, intersect_end_points, circle_scanned=get_curve(obstacles, cur_state, new_target, rs)
        # check the distance
        min_dis=get_closest_distance(cur_state, circle_scanned)
        # todo change the motion_mode early
        # if is_intersect and min_dis<self.r_tan:
        #     self.motion_mode=1
        #     # print log info
        #     # print("intersect_end_points:",intersect_end_points)
        #     # print("new_target:", new_target)
        #     # print("cur_state:", cur_state)
        # else:
        #     # free space exploration
        #     self.motion_mode=0
        # todo check the condition with more examples
        inside_obs=self.check_inside_obs(time, mymap)
        if inside_obs and min_dis< self.rs/2:
            self.motion_mode=1
        else:
            self.motion_mode=0

        return self.motion_mode


    def tangent_bug_planner(self, time, mymap):
        start_point=self.state[time, :2]
        inner_states = start_point  # the state array will be appended
        goal_point = self.target[time]
        obstacles = mymap.obstacles
        rs = self.rs
        r_tan=self.r_tan
        step_len = self.step_len
        inner_time = 0
        time_limit = 400  # end the simulation with time limit
        mode = 0  # mode=0 motion to goal, mode=1 boundary follow
        # get the intersection curve and endpoints
        followed_curve = np.zeros(2)
        hit_times=0
        hit_point = []
        hit_time = 0
        boundary_follow_finished = False
        previous_angle=0
        # start the main while loop

        while True:

            if inner_time == 0:
                cur_state = inner_states
            else:
                cur_state = inner_states[inner_time]

            is_intersect, end_points, scanned_curve = get_curve(obstacles, cur_state, goal_point, rs)
            uniqe_scanned_curve = list(map(list, set(map(tuple, scanned_curve))))
            followed_curve = np.vstack((followed_curve, uniqe_scanned_curve))
            followed_curve = list(map(list, set(map(tuple, followed_curve))))

            # print("end points", end_points)

            if not is_intersect or mode==0:
                # print("not intersect")
                temp_goal = goal_point
            else:
                temp_goal = get_heuristic_goal(cur_state, goal_point, end_points)
            # print("temp goal", temp_goal)

            if mode == 0:  # go straight to goal
                new_state = go_straight(cur_state, temp_goal, step_len)
                mode = check_along(new_state, cur_state, scanned_curve, goal_point, r_tan, temp_goal)
                if mode :
                    if hit_times==0:
                        print("end motion to goal, start boundary following")
                        hit_point = new_state  # store the boundary following point
                        # print("hit point", hit_point)
                        hit_time = inner_time
                        # print("hit time", hit_time)
                        hit_times+=1
                    dy = temp_goal[1] - new_state[1]
                    dx = temp_goal[0] - new_state[0]
                    previous_angle = atan2(dy, dx)
            else:
                new_state, new_angle = boundary_follow(previous_angle, cur_state, temp_goal, step_len, scanned_curve)
                previous_angle = new_angle
                mode = check_off(new_state, scanned_curve, goal_point, rs, obstacles, step_len, end_points)

                if mode == 0:
                    print("end boundary following, start motion to goal ")


            inner_states = np.vstack((inner_states, new_state))




            print("time step:", inner_time)

            if mode == 1 and np.linalg.norm(new_state - hit_point) < 0.2*rs and inner_time - hit_time > 10:
                boundary_follow_finished = True
                self.boundary_follow_finished = True

            if (new_state == goal_point).all():
                print("Goal reached")
                self.boundary_follow_finished=True
                break
            elif inner_time > time_limit:
                print("Time limit reached")
                break
            elif boundary_follow_finished:
                print("Boundary following finished")

                break
            else:
                inner_time += 1

        # calculate the boundary following time

        tangent_start_time=time
        tangent_dauration=len(inner_states)-1
        tangent_end_time=tangent_start_time+tangent_dauration

        # update the value
        self.tangent_end_time=tangent_end_time
        self.tangent_start_time=tangent_start_time
        self.tangent_targets=inner_states
        self.inside_tangent_planner=True
        self.followed_curve=followed_curve

        return inner_states, boundary_follow_finished, followed_curve


    # check if target inside obstacle
    # def check_inside_obstacle(self, cur_target, mymap):
    #     x=cur_target[0]
    #     y=cur_target[1]
    #     row=y
    #     pass



    def update_state_tangent(self, time, target):

        cur_state=self.state[time, :2]
        cur_v=self.state[time,2:]
        cur_target=target
        u_gamma = -gv.c1_gamma * (cur_state - cur_target) - gv.c2_gamma * cur_v

        # limit gamma attraction
        norm_u_gamma = np.linalg.norm(u_gamma)
        if norm_u_gamma > 150:
            u_gamma = 150 * u_gamma / norm_u_gamma

        # check the motion mode and neighbour agent mode
        # calculate the repulsive force from neighbour agents :u_alpha
        # todo

        # calculate the deviation
        d_v = u_gamma * gv.step_size
        d_position = cur_v* gv.step_size
        norm_d_position=np.linalg.norm(d_position)
        if norm_d_position>self.step_len:
            d_position=self.step_len/norm_d_position*d_position

        # add new state vector
        self.state[time + 1, :2] = cur_state + d_position
        self.state[time + 1, 2:] = cur_v + d_v

    def get_target_from_tangent(self, time):

        inner_time=time-self.tangent_start_time
        cur_target=self.tangent_targets[inner_time]
        self.target[time]=cur_target
        self.target[time+1]=self.tangent_targets[inner_time+1]

        return cur_target


    def reset_tangent(self):
        self.tangent_end_time = 0
        self.tangent_start_time = 0
        self.inside_tangent_planner = False
        self.tangent_targets = []
        self.boundary_follow_finished=False
        self.motion_mode=0
        self.followed_curve = []


    def check_bug_neighbour(self, time):
        q_i=self.state[time,:2]
        meet_distance=self.rc/2
        flag=0
        traveled_time_i=time-self.tangent_start_time
        for r in self.neighbour:
            q_j=r.state[time,:2]
            traveled_time_j=time-r.tangent_start_time
            time_d=traveled_time_i-traveled_time_j
            distance=np.linalg.norm(q_i-q_j)
            if r.motion_mode and distance<meet_distance and time_d<=0:
                flag=1
                print("robots meet and leave boundary following mode ")
                return True
        if not flag:
            return False


    def avoid_robot(self, time):
        """
        Check the neighbour alpha agent
        detect the closest beta obstacle
        compare the boundary following time
        the smaller one receive repulsive force
        the smaller one update the state
        the motion mode is reset to 0

        Args:
            time: the simulation time

        Returns:

        """
        alph_neighbour=self.neighbour
        travel_time_self = time - self.tangent_start_time
        for r in alph_neighbour:
            travel_time_neighbour = time - r.tangent_start_time
            t_d=travel_time_self-travel_time_neighbour
            if r.motion_mode and t_d<0:
                self.motion_mode=0
                self.repulsive_force(time)
                break



    def repulsive_force(self, time):
        u_alpha=np.zeros(2)
        u_beta=np.zeros(2)

        neighbour = self.neighbour
        A = m.adjacency_alpha(gv.robotList)
        i = self.id
        # position of the robot i at time t
        q_i = self.state[time, :2]
        # velocity of robot i
        v_i = self.state[time, 2:]

        for robot_j in neighbour:
            j = robot_j.id
            a_ij = A[i, j]
            # position and velocity of robot j
            q_j = np.array(robot_j.state)[time, :2]
            v_j = np.array(robot_j.state)[time, 2:]
            u_alpha_j_1 = gv.c1_alpha * m.phi_alpha(m.sigma_norm(q_j - q_i)) * m.norm_direction(q_i, q_j)
            u_alpha_j_2 = gv.c2_alpha * a_ij * (v_j - v_i)
            u_alpha += u_alpha_j_1 + u_alpha_j_2

        q_beta=self.get_beta_agent(time)
        print("q_beta:", q_beta)
        # the velocity of beta agent is [0,0]
        # the position of beta agent is q_beta, from self.get_beta_agent(time)
        v_beta=np.zeros(2)
        b_ik=m.bump_beta(m.sigma_norm(q_beta-q_i)/m.sigma_norm(gv.d_beta))
        u_beta_k_1 = gv.c1_beta * m.phi_beta(m.sigma_norm(q_beta - q_i)) * m.norm_direction(q_beta, q_i)
        #         # u_beta_k_2 = np.multiply((v_beta - v_i),gv.c2_beta * b_ik )
        u_beta_k_2 = (gv.c2_beta * b_ik) * (v_beta - v_i)
        u_beta = u_beta_k_1 + u_beta_k_2
        u=u_alpha+u_beta
        print("u repulsive:", u)

        norm_u = np.linalg.norm(u)
        if norm_u > 20:
            u = 20 * u/ norm_u

        cur_position = self.state[time, :2]
        cur_v = self.state[time, 2:]

        # calculate the deviation
        d_v = u * gv.step_size * gv.rate
        d_position = self.state[time, 2:] * gv.step_size * gv.rate
        norm_d_position = np.linalg.norm(d_position)
        if norm_d_position > self.d_position_limit:
            d_position = self.d_position_limit * d_position / norm_d_position

        self.state[time + 1, :2] = cur_position + d_position
        self.state[time + 1, 2:] = cur_v + d_v

    def reset_target(self, time):

        # get the benefit value matrix
        # b_matrix=self.benefit_matrix
        # row=len(b_matrix)
        # col =len(b_matrix[0])
        # min_b_value=100
        #
        # # get the smallest value from the matrix
        # for x in range(col):
        #     for y in range(row):
        #         if 0<b_matrix[y,x]<min_b_value:
        #             min_b_value=b_matrix[y,x]
        #             target=[x,y]
        # target_x=gv.grid_length*target[0]+gv.grid_length/2
        # target_y=gv.grid_length*target[1]+gv.grid_length/2
        # self.target[time+1]=[target_x, target_y]

        # reset target along the negative direction of the current velocity
        cur_v=self.state[time,2:]
        cur_state=self.state[time,:2]
        negative_v=-cur_v/np.linalg.norm(cur_v)
        step_len=2*self.rs
        temp_target=step_len*negative_v+cur_state

        # limit the target point inside the boundary
        if gv.x_bound<temp_target[0] or temp_target[0]<0:
            temp_target[0]=gv.x_bound*0.8
        if gv.y_bound<temp_target[1] or temp_target[1]<0:
            temp_target[1] = gv.y_bound * 0.8

        self.target[time+1]=temp_target



    def check_new_obstacle(self):
        """
        check if a nes obstacle is discovered
        if new_obstacle found clean the self.followed_curve

        Returns:

        """
        pass

    def scann_rs(self):
        """
        scann the surrounding area
        Returns: obstacle points

        """
        pass


    def get_beta_agent(self, time):
        # robot current sate
        q = self.state[time, :2]

        # find the closest point on obstacle
        close_distance = self.rs
        close_point=[]
        for x in range(gv.x_n):
            for y in range(gv.y_n):
                if self.tarobsmap[x, y] == -1:
                    x_center = gv.grid_length / 2 + x * gv.grid_length
                    y_center = gv.grid_length / 2 + y * gv.grid_length
                    cell_center = np.array([x_center, y_center])
                    distance = np.linalg.norm(q - cell_center)
                    # update the closest point
                    # set the closest point as beta neighbour
                    if distance < close_distance:
                        close_distance = distance
                        close_point = [x_center, y_center]
                        return True
        return False

    def check_nearby_obs(self, time):
        # robot current sate
        q = self.state[time, :2]

        # find the closest point on obstacle
        close_distance = self.rs*1.3
        close_point=[]
        for x in range(gv.x_n):
            for y in range(gv.y_n):
                if self.tarobsmap[x, y] == -1:
                    x_center = gv.grid_length / 2 + x * gv.grid_length
                    y_center = gv.grid_length / 2 + y * gv.grid_length
                    cell_center = np.array([x_center, y_center])
                    distance = np.linalg.norm(q - cell_center)
                    # update the closest point
                    # set the closest point as beta neighbour
                    if distance < close_distance:
                        close_distance = distance
                        close_point = [x_center, y_center]
                        return True
        return False


    # get the nearest beta point on k obstacle around the robot
    def get_beta(self, time):
        grid = self.tarobsmap
        rows, cols = len(grid), len(grid[0])
        visit = set()
        beta_neighbour = []
        neighbour=[]
        grid_length = gv.grid_length
        rs = self.rs

        # the position and velocity of the robot
        position = self.state[time, :2]
        velocity = self.state[time, 2:]

        def get_center(row, col):
            x_center = grid_length / 2 + col * grid_length
            y_center = grid_length / 2 + row * grid_length
            cell_center = np.array([x_center, y_center])
            return cell_center

        def bfs(r, c):
            q = collections.deque()
            visit.add((r, c))
            q.append((r, c))
            min_distance = rs
            nearest_neighbour = []

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
            p_a = np.array(p)
            p_b = np.array(p_beta)
            # identity matrix with dimension 2
            I = np.identity(self.dimension)
            # a_k is unit normal vector
            a_k = (p_a - p_b) / np.linalg.norm(p_a - p_b)
            # projection matrix
            proj_matrix = I - np.outer(a_k, a_k.T)
            # project velocity
            v_proj = np.dot(v, proj_matrix)
            return v_proj

        for r in range(rows):
            for c in range(cols):
                if grid[r][c] == -1 and (r, c) not in visit:
                    beta_neighbour.append(bfs(r, c))

        for beta in beta_neighbour:
            if np.any(beta):
                neighbour.append(beta)

        new_neighbour=[]
        for p_beta in neighbour:
            v_proj = v_projection(position, p_beta, velocity)
            # add the velocity to the vector
            p_beta = np.append(p_beta, v_proj)
            new_neighbour.append(p_beta)
            # p_beta = np.append(p_beta, v_proj[1])

        self.beta_neighbour = new_neighbour

        return new_neighbour

    # todo: integrate tangent bug to the Robot Class
    def t_bug(self, cur_state, new_targt, mymap):
        """
        Args:
            cur_state: the current state
            new_targt: the target point returned by update_target()
            mymap: the map
        Returns:
            temp_target and new_state
        """
        # integrate the tangent_bug here
        pass


def get_middle_point(followed_curve):

    min_x = min([sub[0] for sub in followed_curve])
    max_x = max([sub[0] for sub in followed_curve])
    min_y = min([sub[1] for sub in followed_curve])
    max_y = max([sub[1] for sub in followed_curve])
    middle_x=(min_x+max_x)/2
    middle_y=(min_y+max_y)/2
    sc = int(middle_x // gv.grid_length)
    sr = int(middle_y // gv.grid_length)
    return sr, sc

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

def show_robot(robotList, mymap):
    figure0 = plt.figure('robot initial state ', figsize=(5, 5))
    for robot in robotList:
        plt.scatter(robot.initial_state[0], robot.initial_state[1])
        plt.scatter(robot.initial_target[0], robot.initial_target[1], marker='*', color='black')
        plt.annotate(robot.id, (robot.initial_state[0], robot.initial_state[1]))
        plt.annotate(robot.id, (robot.initial_target[0], robot.initial_target[1]))
        print('initial state:', robot.initial_state)
        print('initial target:', robot.initial_target)

    obstacles=mymap.obstacles
    for obstacle in obstacles:
        plt.plot(obstacle[0], obstacle[1])

    plt.xlim(0, gv.x_bound )
    plt.ylim(0, gv.y_bound )
    plt.title('The initial state of robots')
    # add x-label
    plt.xlabel('x-coordinate')
    # add y label
    plt.ylabel('y-coordinate')
    # save image to the image folder
    # plt.savefig('../image/initial_state.png')
    plt.show()
    plt.savefig('../image/initial_state.png')


def show_infomap(robot, mymap):
    '''

    Args:
        robot: robot object
        mymap: the env_map object

    Returns:
        plot the information map of that robot

    '''
    for x in range(gv.x_n):
        for y in range(gv.y_n):
            if robot.tarobsmap[y,x]==-1:
                robot.infomap[y,x]=-1

    # flip the information map
    # flip_infomap=np.flipud(robot.infomap)

    # define the color map
    color_map = {1: np.array([255, 51, 51]),  # 1 is visited area filled with red
                 0: np.array([0, 102, 204]),  # 0 is free space filled with blue color
                 -1: np.array([192,192,192])}  # -1 is obstacle filled with gray color

    # define a sD matrix to store the color value
    data_3d = np.ndarray(shape=(gv.y_n, gv.x_n, 3), dtype=int)


    # plot the grid_map with color
    for i in range(gv.y_n):
        for j in range(gv.x_n):
            data_3d[i][j] = color_map[robot.infomap[i][j]]

    # plot the obstacles on the map
    for obstacle in mymap.obstacles:
        plt.plot(obstacle[0], obstacle[1])

    # figure = plt.figure('2D grid map', figsize=(5, 5))
    # add label
    plt.xlabel("X coordinate [m]")
    plt.ylabel("Y coordinate [m]")
    plt.title("Information map of robot %i" % robot.id)

    # show image
    plt.imshow(data_3d, origin='lower')
    plt.show()


def show_merge_infomap(robotList):
    '''
    Args:
        robotList: a list of robot

    Returns:
        merge the info maps and return a percent in [0,1]
        todo return also the merged_info map matrix
    '''

    merged_map=np.zeros((gv.y_n,gv.x_n))
    for robot in robotList:
        # logic_or two matrix and convert it into number 0 and 1
        merged_map=1*np.logical_or(merged_map,robot.infomap)

    coverage_area = np.sum(merged_map)
    area = gv.x_n * gv.y_n
    percent = coverage_area / area

    # flip_infomap = np.flipud(merged_map)

    # define the color map
    color_map = {1: np.array([255, 51, 51]),  # 1 is visited area filled with red
                 0: np.array([0, 102, 204])}  # 0 is free space filled with blue color

    # define a sD matrix to store the color value
    data_3d = np.ndarray(shape=(gv.y_n, gv.x_n, 3), dtype=int)

    # plot the grid_map with color
    for i in range(gv.y_n):
        for j in range(gv.x_n):
            data_3d[i][j] = color_map[merged_map[i][j]]

    # add label
    plt.xlabel("X coordinate [m]")
    plt.ylabel("Y coordinate [m]")
    plt.title("Merged Information Map with Coverage Percent %1.3f " % percent)

    # show image
    plt.imshow(data_3d, origin='lower')


def merge_infomap(robotList):
    '''
    Args:
        robotList: a list of robot
    Returns:
        merge the info maps and return a percent in [0,1]
    '''
    # todo consider tarobsmap
    merged_map = np.zeros((gv.y_n, gv.x_n))
    for robot in robotList:
        # logic_or two matrix and convert it into number 0 and 1
        merged_map = 1 * np.logical_or(merged_map, robot.infomap)

    coverage_area = np.sum(merged_map)
    area = gv.x_n * gv.y_n
    percent = coverage_area / area

    return percent


def show_coverage_percent(c_percent):
    '''
    Args:
        c_percent: a lsit of coverage_percent number
    Returns:
        plot the coverage percent changes
    '''
    figure=plt.figure('Coverage percent',figsize=(5,5))
    time_steps=np.linspace(0,gv.T,gv.Iteration-1)
    plt.plot(time_steps,c_percent)
    plt.xlabel("Simulation time")
    plt.ylabel("coverage percent")
    plt.title("Area Coverage Percent")
    plt.savefig('../image/coverage_percent.png')






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
    benefit_value = gv.robotList[1].benefit_value(1)
    print(benefit_value)
    print(len(benefit_value))
    print(len(benefit_value[0]))

    # test update infomap
    mymap = env.EnvMap(300, 300, 1)
    mymap.add_circle(90, 60, 45)
    gv.env_map = mymap.grid_map
    gv.robotList[1].update_info_map(1,mymap)
    print(gv.robotList[1].infomap)

    # gv.robotList[1].get_beta(1)
    # print(gv.robotList[1].beta_neighbour)
    robotList = define_robot(3)





