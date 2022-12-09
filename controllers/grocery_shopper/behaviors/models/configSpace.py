import numpy as np
from scipy.signal import convolve2d
import matplotlib.pyplot as plt

class ConfigSpace:
    def __init__(self, kernel_size=(5,5)):
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

    def convolve(self, map):
        """Convolve will make the obstacle bounds bigger to avoid some collision"""
        q25 = np.percentile(map, 25)
        map = self.denoise(map,q25)
        map = convolve2d(map, self.kernel, mode='same')
        map[map > np.subtract(*np.percentile(map, [75, 25]))] = 1
        return map

    def denoise(self, map, ftol):
        """
        Remove the wave-like lidar reading cause by the robot turning causing overlaps.
        To combat this noise that accumulates over time, we've also disable the lidar 
        readings when turning with negative and position spin.
        """
        grad = convolve2d(map, self.horizontal_mask, mode='same')
        grad += convolve2d(map, self.vertical_mask, mode='same')
        idx = grad >= ftol
        grad[~idx] = 0
        grad[idx] = map[idx]
        grad[idx] = np.clip(grad[idx], 0.0, 1.0)
        return grad

    def filter_small_islands(self, map, size=25):
        """
        Should be used in small amounts like 3 times. If used to many times
        the map will degrade in quality. Calcuates the blobs of the images
        and remove the ones that consists of size pixels or less. This will in general
        enhance the final mapping.
        """
        def expand(img_mask, cur_coord):
            coordinates_in_blob = []
            coordinate_list = [cur_coord]
            while len(coordinate_list) > 0:
                (y, x) = coordinate_list.pop()
                if y < 0 or x < 0 or y >= img_mask.shape[0] or x >= img_mask.shape[1]:
                    continue
                if img_mask[y, x] == 0.0:
                    continue
                img_mask[y,x] = 0
                coordinates_in_blob.append([x,y])
                coordinate_list.extend([(y-1, x),(y+1, x),(y, x-1),(y, x+1)])
            return np.asarray(coordinates_in_blob)

        def get_blobs(img_mask):
            img_mask_height, img_mask_width = img_mask.shape[0], img_mask.shape[1]
            blobs_list, blobs_size = [], []
            img_mask_copy = img_mask.copy()
            for idx in range(img_mask_height*img_mask_width):
                y, x = idx // img_mask_width, idx % img_mask_width
                if (img_mask[y][x] > 0):
                    blob_coords = expand(img_mask_copy, [y, x])
                    if (len(blob_coords)):
                        blobs_list.append(blob_coords)
                        blobs_size.append(len(blob_coords))
            return np.array(blobs_list), np.array(blobs_size)
        
        blobs_list, blobs_size = get_blobs(map.copy())
        outlier_idx = np.where(blobs_size < size)
        outliers = blobs_list[outlier_idx]
        if (outliers != []):
            for x, y in np.vstack(outliers):
                map[y][x] = 0
        return map