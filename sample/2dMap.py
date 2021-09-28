
'''
define 2D map
enable adding different shape of obstacles
circle, polygon, irregular shape and walls
'''
import numpy as np
import matplotlib.pyplot as plt


class EnvMap:
    def __init__(self, x_bound, y_bound, grid_length):
        self.x_bound=x_bound
        self.y_bound=y_bound
        self.grid_length=grid_length
        self.x_n=self.x_bound/self.grid_length
        self.y_n=self.y_bound/self.grid_length
        self.grid_map=[]

    @classmethod
    def make_grid(cls,row,col):
        grid_map=np.zeros((cls.row,cls.col))


    def add_circle(self):
        pass

    def add_polygon(self):
        pass

    def add_irregular(self):
        pass







