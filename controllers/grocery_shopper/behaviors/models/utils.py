"""
utils.py

Last updated on Fri Dec 9 2022
@Lead: Joshua Truong
"""


import numpy as np
import matplotlib.pyplot as plt

class Pose:
    def __init__(self, x=0, y=0, theta=0):
        self.x     = x
        self.y     = y
        self.theta = theta

    def __str__(self):
        return str(self.__dict__)

class Map:
    def __init__(self, size=(360, 360)):
        self.shape = size
        self.map = np.zeros(size)

    def pixel(self, x, y):
        return self.map[y][x]

    def update_pixel(self, x, y, val):
        self.map[y][x] = val
    
    def update_map(self, map):
        self.map = map.astype('float64')