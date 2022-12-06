import numpy as np
from scipy.signal import convolve2d


class ConfigSpace:
    def __init__(self, kernel_size=(10,10)):
        self.kernel = np.ones(kernel_size)

    def run(self, map):
        map = convolve2d(map, self.kernel, mode='same')
        map[map > 0] = 1
        return map