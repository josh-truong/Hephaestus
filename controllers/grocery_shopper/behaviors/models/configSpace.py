import numpy as np
from scipy.signal import convolve2d
import matplotlib.pyplot as plt

class ConfigSpace:
    def __init__(self, kernel_size=(10,10)):
        self.kernel = np.ones(kernel_size)
        self.horizontal_mask = np.array([
            [-1, -1, -1],
            [ 2,  2,  2],
            [-1, -1, -1],
        ])
        self.vertical_mask = np.array([
            [-1, 2, -1],
            [-1, 2, -1],
            [-1, 2, -1],
        ])

    def run(self, map):
        q25 = np.percentile(map, 25)
        map = self.filter_by_convolving(map,q25)
        map = convolve2d(map, self.kernel, mode='same')
        map[map > np.subtract(*np.percentile(map, [75, 25]))] = 1
        return map

    def filter_by_convolving(self, map, ftol):
        grad = convolve2d(map, self.horizontal_mask, mode='same')
        grad += convolve2d(map, self.vertical_mask, mode='same')
        idx = grad >= ftol
        grad[~idx] = 0
        grad[idx] = map[idx]
        grad[idx] = np.clip(grad[idx], 0.0, 1.0)
        return grad