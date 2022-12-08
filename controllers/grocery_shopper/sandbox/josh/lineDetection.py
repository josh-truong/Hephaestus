
import numpy as np
import matplotlib.pyplot as plt

import numpy as np
from scipy.signal import convolve2d
import matplotlib.pyplot as plt




    # def remove_outliers(self, img):
    #     blobs_list, blobs_size = self.return_blobs(img.copy())
    #     blobs_list, blobs_size = np.array(blobs_list), np.array(blobs_size)
    #     idx = np.where(blobs_size < 50)
    #     outliers = np.row_stack(blobs_list[idx])
    #     img[outliers[:,1], outliers[:,0]] = 0
    #     return img

    # def return_blobs(self, img):
    #     """Return blobs of connected pixels"""
    #     def expand(img_mask, cur_coord):
    #         coordinates_in_blob = []
    #         coordinate_list = [cur_coord]
    #         while len(coordinate_list) > 0:
    #             (y, x) = coordinate_list.pop()
    #             if y < 0 or x < 0 or y >= img_mask.shape[0] or x >= img_mask.shape[1]:
    #                 continue
    #             if img_mask[y, x] == 0.0:
    #                 continue
    #             img_mask[y,x] = 0
    #             coordinates_in_blob.append([x,y])
    #             coordinate_list.extend([[y-1, x],[y+1, x],[y, x-1],[y, x+1]])
    #         return coordinates_in_blob

    #     img_height, img_width = img.shape[0], img.shape[1]
    #     blobs_list, blobs_size = [], []
    #     img_copy = img.copy()
    #     for idx in range(img_height*img_width):
    #         y, x = idx // img_width, idx % img_width
    #         if (img[y][x] > 0):
    #             blob_coords = expand(img, [y, x])
    #             if (len(blob_coords)):
    #                 blobs_list.append(blob_coords)
    #                 blobs_size.append(len(blob_coords))
    #     return blobs_list, blobs_size