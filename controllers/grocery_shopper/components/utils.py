import numpy as np
import matplotlib.pyplot as plt

class Pose:
    def __init__(self, x, y, theta):
        self.x     = x
        self.y     = y
        self.theta = theta


class Map:
    def __init__(self, size=(360, 360)):
        self.shape = size
        self.map = np.zeros(size)

    def pixel(self, x, y):
        return self.map[y][x]

    def update_pixel(self, x, y, val):
        self.map[y][x] = val
    
    def update_map(self, map):
        self.map = map

    def save(self, filename="map.npy"):
        np.save("assets/" + filename, self.map)
        print("Map file saved.")

    def load(self, filename="map.npy"):
        self.map = np.load("assets/" + filename, allow_pickle=True)
        print("Map file loaded.")
        return self.map
    
    def filter(self, map=None, tol=0.5, filename="filter_map.npy"):
        map = self.map if (map is None) else map
        map = (map >= tol).astype(int)
        np.save("assets/" + filename, map)
        print("Filter map file saved.")
        return map
    
    def display(self, map=None):
        map = self.map if (map is None) else map
        plt.imshow(map)
        plt.colorbar()
        plt.show()
