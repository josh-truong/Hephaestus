import numpy as np
from collections import defaultdict
import matplotlib.pyplot as plt
from scipy.signal import convolve2d

class EdgeDetection():
    def __init__(self, m, frequency=25):
        self.m = m
        self.count = 0
        self.frequency = frequency

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

    def run(self):
        self.count += 1
        if (self.count%self.frequency != 0): return

        map = self.m.Mapping.Map.map
        map = self.convolve(map)
        # map = self.remove_small_islands(map)
        self.m.Device.redraw_display(map)
        self.m.Mapping.get_map_variance()

        # print("Map updated.")

    def convolve(self, map, ftol=1):
        grad = convolve2d(map, self.horizontal_mask, mode='same')
        grad += convolve2d(map, self.vertical_mask, mode='same')
        idx = grad >= ftol
        grad[~idx] = 0
        grad[idx] = map[idx] + 0.05
        grad[idx] = np.clip(grad[idx], 0.0, 1.0)
        return grad

    def remove_small_islands(self, map):
        (x, y) = np.where(map > 0)
        non_zero_pixels = list(zip(x, y))
        pixels = dict.fromkeys(non_zero_pixels, -1)
        
        num_blobs = 0
        while( len(non_zero_pixels) > 0 ):
            pixel = non_zero_pixels.pop()
            x, y = pixel

            if (pixels[pixel] == -1):
                num_blobs += 1
                pixels[pixel] = num_blobs
            
            top    = (x, y-1)
            right  = (x+1, y)
            bottom = (x, y-1)
            left   = (x-1, y)

            if top in non_zero_pixels:
                pixels[top] = num_blobs
            if right in non_zero_pixels:
                pixels[right] = num_blobs
            if bottom in non_zero_pixels:
                pixels[bottom] = num_blobs
            if left in non_zero_pixels:
                pixels[left] = num_blobs
        
        blobs = defaultdict(list)
        for key, value in pixels.items():
            blobs[value].append(key)
        
        print(sorted([len(blob) for blob in blobs.values()]))
        return map
