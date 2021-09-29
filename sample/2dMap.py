
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
                if distance<=r:
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

    def add_line(self,p1,p2):
        '''
        The method is based on Bresenham's line algorithm

        Args:
            p1: the starting point [x1,y1]
            p2: the ending point [x2,y2]

        Returns: add line on self.grid_map

        '''
        x1,y1=p1
        x2,y2=p2
        dx=x2-x1
        dy=y2-y1






    def show_map(self):
        '''

        Returns: a colored map in which obstacle is black
        and uncovered space is blue

        '''

        color_map={1:np.array([0,0,0]),
                   0:np.array([102,178,255])}
        data_3d=np.ndarray(shape=(self.x_n,self.y_n,3),dtype=int)
        for i in range(self.x_n):
            for j in range(self.y_n):
                data_3d[i][j]=color_map[self.grid_map[i][j]]
        plt.imshow(data_3d)
        plt.show()






map=EnvMap(300,300,1)
map.add_circle(60,60,40)
map.show_map()





