'''
define 2D map
enable adding different shape of obstacles
circle, polygon, irregular shape and walls
'''
from math import *
from typing import List

import matplotlib.pyplot as plt
import numpy as np

import global_value as gv


class EnvMap:
    def __init__(self, x_bound, y_bound, grid_length):
        self.x_bound = x_bound
        self.y_bound = y_bound
        self.grid_length = grid_length
        self.x_n = int(self.x_bound / self.grid_length)  # number of grid in x direction
        self.y_n = int(self.y_bound / self.grid_length)  # number of grid in y direction
        self.grid_map = self.make_grid(self.x_n, self.y_n)
        gv.env_map = self.grid_map
        self.obstacles = []  # store the obstacle points
        self.point_num = 1000  # number of points on a obstacle
        self.circle_bound=[] # [x,y,r]
        self.polygon_bound=[] # [x_min, x_max, y_min, y_max]

    def make_grid(self, x_n, y_n):
        grid = np.zeros((y_n, x_n))
        # grid[:,0]=1
        # grid[0,:]=1
        # grid[:,-1]=1
        # grid[-1,:]=1

        return grid

    def add_circle(self, x_position, y_position, r):
        '''
        Args:
            x_position: center point of the circle in x coordinate
            y_position: center point of the circle in y coordinate
            r: radium of the circle

        Returns: No return, just modify grid_map
        '''

        # get the grid_length from class attribute
        grid_length = self.grid_length

        # tolerance of the circle radius
        r_t = r / 20

        # set grid point near the circle boundary to 1
        for i in range(self.x_n):
            for j in range(self.y_n):
                center_point = [grid_length / 2 + i * grid_length, grid_length / 2 + j * grid_length]
                distance = sqrt((center_point[0] - x_position) ** 2 + (center_point[1] - y_position) ** 2)
                if abs(distance - r) < grid_length:
                    self.grid_map[j, i] = 1

        # create dense points on the circle
        t = np.linspace(0, 2 * pi, self.point_num)
        circle = np.zeros((2, self.point_num))
        for i in range(self.point_num):
            circle[0, i] = x_position + r * cos(t[i])
            circle[1, i] = y_position + r * sin(t[i])

        # add circle points to self.obstacles
        self.obstacles.append(circle)
        self.circle_bound.append([x_position, y_position, r])

    def add_polygon(self, key_points: List[int]):
        '''

        Args:
            key_points: the vertex of the polygon [x1, y1, x2, y2, x3, y3]

        Returns: no return, the self.grid_map is modified

        '''
        # append the first key_point to the end of the list
        key_points.append(key_points[0])
        key_points.append((key_points[1]))
        poly_lines = []

        # calculate the number of points
        number_of_points = int(len(key_points) / 2 - 1)

        # iterate through the key_points and call add_line function
        for i in range(number_of_points):
            start_point = [key_points[2 * i], key_points[2 * i + 1]]
            end_point = [key_points[2 * i + 2], key_points[2 * i + 3]]
            line_i = self.add_line(start_point, end_point)
            if len(poly_lines) == 0:
                poly_lines = line_i
            else:
                poly_lines = np.concatenate((poly_lines, line_i), axis=1)

        self.obstacles.append(poly_lines)
        x_min=min([key_points[0],key_points[2], key_points[4], key_points[6],key_points[8],key_points[10]])
        x_max=max([key_points[0],key_points[2], key_points[4], key_points[6],key_points[8],key_points[10]])
        y_min = min([key_points[1], key_points[3], key_points[5], key_points[7],key_points[9],key_points[11]])
        y_max = max([key_points[1], key_points[3], key_points[5], key_points[7],key_points[9],key_points[11]])
        self.polygon_bound.append([x_min, x_max, y_min, y_max])

    def add_line(self, p1: List, p2: List):
        '''

        Args:
            p1: start point [x1,y1]
            p2: end point [x2,y2]

        Returns: modify the grid_map

        '''

        x1, y1 = p1[0] // self.grid_length, p1[1] // self.grid_length
        x2, y2 = p2[0] // self.grid_length, p2[1] // self.grid_length
        dx = x2 - x1
        dy = y2 - y1

        is_steep = abs(dy) > abs(dx)

        # Rotate line
        if is_steep:
            x1, y1 = y1, x1
            x2, y2 = y2, x2

        # Swap start and end points if necessary and store swap state
        swapped = False
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
            swapped = True

        # Recalculate differentials
        dx = x2 - x1
        dy = y2 - y1

        # Calculate error
        error = int(dx / 2.0)
        ystep = 1 if y1 < y2 else -1

        #  generate points between p1 and p2
        y = y1
        points = []
        for x in range(x1, x2 + 1):
            coord = (y, x) if is_steep else (x, y)
            points.append(coord)
            error -= abs(dy)
            if error < 0:
                y += ystep
                error += dx

        # Reverse the list if the coordinates were swapped
        if swapped:
            points.reverse()

        # print the line on the grid_map
        for point in points:
            self.grid_map[point[1], point[0]] = 1

        line = np.zeros((2, self.point_num))
        for i in range(self.point_num):
            line[0, i] = p1[0] + i / self.point_num * (p2[0] - p1[0])
            line[1, i] = p1[1] + i / self.point_num * (p2[1] - p1[1])

        return line

    # def floodFill(self, sr, sc, newColor):
    #
    #     R = len(self.grid_map)
    #     C = len(self.grid_map[0])
    #     color = self.grid_map[sr][sc]
    #     if color == newColor: return
    #     def dfs(r, c):
    #         if self.grid_map[r][c] == color:
    #             self.grid_map[r][c] = newColor
    #             if r >= 1:
    #                 dfs(r - 1, c)
    #             if r + 1 < R:
    #                 dfs(r + 1, c)
    #             if c >= 1:
    #                 dfs(r, c - 1)
    #             if c + 1 < C:
    #                 dfs(r, c + 1)
    #
    #     dfs(sr, sc)
    #     return

    def show_map(self):
        '''

        Returns: a colored map in which obstacle is black
        and uncovered space is blue

        '''

        # define the color map
        color_map = {1: np.array([0, 0, 0]),  # 1 is obstacle filled with black color
                     0: np.array([0, 102, 204])}  # 0 is free space filled with blue color

        # define a sD matrix to store the color value
        data_3d = np.ndarray(shape=(self.y_n, self.x_n, 3), dtype=int)

        # flip the map
        # flip_grid_map=np.flipud(self.grid_map)
        # plot the grid_map with color
        for i in range(self.x_n):
            for j in range(self.y_n):
                data_3d[j][i] = color_map[self.grid_map[j][i]]

        figure1 = plt.figure('2D grid map', figsize=(5, 5))
        # add label
        plt.xlabel("X coordinate [m]")
        plt.ylabel("Y coordinate [m]")
        plt.title("2D grid map")

        # show image
        plt.imshow(data_3d, origin='lower')

        # show the obstacle points
        for obstacle in self.obstacles:
            plt.plot(obstacle[0], obstacle[1],linewidth=2, color='r')
        plt.savefig('../image/env_map_example.png')
        plt.show()



if __name__ == "__main__":
    mymap = EnvMap(50,50, 1)

    # mymap.add_circle(30, 20, 10)
    mymap.add_circle(10, 15, 6)
    # mymap.add_circle(11,20,4)
    # mymap.add_polygon([20,20,25,20,25,25,20,25])
    # # mymap.floodFill(90,60,1)
    # add concave obstacles
    mymap.add_polygon([24,41,37,37,40,27,32,23,28,30,20,32])
    # mymap.add_polygon([15, 22, 16, 15,20,12,10,7,6,16])
    # mymap.add_polygon([20, 37, 33, 33, 37, 23, 28, 19, 24, 26, 16, 28])
    mymap.show_map()

    bounding_box = []
    for obs in mymap.obstacles:
        print(len(obs[0]))
        x_max, y_max = max(obs[0]), max(obs[1])
        print("xmax, ymax", x_max, y_max)
        bounding_box.append([x_max, y_max])

    print(bounding_box)

    # first_obs=mymap.obstacles[0]
    # second_obs=mymap.obstacles[1]
    # third_obs = mymap.obstacles[2]
    # print(len(third_obs[0]))
