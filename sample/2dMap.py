
'''
define 2D map
enable adding different shape of obstacles
circle, polygon, irregular shape and walls
'''
import numpy as np
import matplotlib.pyplot as plt
from typing import List
import math

class EnvMap:
    def __init__(self, x_bound, y_bound, grid_length):
        self.x_bound=x_bound
        self.y_bound=y_bound
        self.grid_length=grid_length
        self.x_n=int(self.x_bound/self.grid_length)
        self.y_n=int(self.y_bound/self.grid_length)
        self.grid_map=self.make_grid(self.x_n,self.y_n)



    def make_grid(self,row, col):

        return np.zeros((row,col))




    def add_circle(self,x_position,y_position,r):
        '''
        Args:
            x_position: center point of the circle in x coordinate
            y_position: center point of the circle in y coordinate
            r: radium of the circle

        Returns: No return, just modify grid_map
        '''

        # get the grid_length from class attribute
        grid_length=self.grid_length

        # calculate center point of each grid
        center_point=[]

        # set grid point inside the circle to 1
        for i in range(self.x_n):
            for j in range(self.y_n):
                center_point=[grid_length/2+i*grid_length,grid_length+j*grid_length]
                distance=math.sqrt((center_point[0]-x_position)**2+(center_point[1]-y_position)**2)
                if distance<=r :
                    self.grid_map[i,j]=1



    def add_polygon(self,key_point:List[int]):
        '''

        Args:
            key_point: the vertex of the polygon

        Returns: no return, the self.grid_map is modified

        '''


        pass

    def add_irregular(self,start_point,matrix):
        '''

        Args:
            start_point:
            matrix:

        Returns:

        '''
        pass



    def add_line(self,p1:List, p2:List):
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
            self.grid_map[point[0], point[1]] = 1

    def floodFill(self, sr, sc, newColor):

        R = len(self.grid_map)
        C = len(self.grid_map[0])
        color = self.grid_map[sr][sc]


        def dfs(r, c):
            if self.grid_map[r][c] == color:
                self.grid_map[r][c] = newColor
                if r >= 1: dfs(r - 1, c)
                if r + 1 < R: dfs(r + 1, c)
                if c >= 1: dfs(r, c - 1)
                if c + 1 < C: dfs(r, c + 1)

        dfs(sr, sc)






    def show_map(self):
        '''

        Returns: a colored map in which obstacle is black
        and uncovered space is blue

        '''

        # define the color
        color_map={1:np.array([0,0,0]), # 1 is obstacle filled with black color
                   0:np.array([102,178,255])} # 0 is free space filled with blue color

        # define a sD matrix to store the color value
        data_3d=np.ndarray(shape=(self.x_n,self.y_n,3),dtype=int)

        # plot the grid_map with color
        for i in range(self.x_n):
            for j in range(self.y_n):
                data_3d[i][j]=color_map[self.grid_map[i][j]]
        plt.imshow(data_3d)
        plt.show()







map=EnvMap(300,300,2)
map.add_circle(90,60,40)
map.add_line([100,100],[160,180])
map.add_line([160,180],[100,250])
map.add_line([100,250],[100,100])

map.show_map()





