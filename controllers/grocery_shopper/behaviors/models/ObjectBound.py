"""
ObjectBound.py

Last updated on Fri Dec 9 2022
@Lead: Joshua Truong
"""
import numpy as np
from matplotlib.patches import Rectangle

class ObjectBound():
    def return_blobs(self, img):
        """Return blobs of connected pixels"""
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
                coordinates_in_blob.append((x,y))
                coordinate_list.extend([[y-1, x],[y+1, x],[y, x-1],[y, x+1]])
            return coordinates_in_blob

        img_height, img_width = img.shape[0], img.shape[1]
        blobs_list = []
        img_copy = img.copy()
        for idx in range(img_height*img_width):
            y, x = idx // img_width, idx % img_width
            if (img[y][x] > 0):
                blob_coords = expand(img, (y, x))
                if (len(blob_coords)):
                    blobs_list.append(blob_coords)
        return blobs_list

    def return_bounds(self, blobs):
        """Calculate the bounds of blobs"""
        bounds = []
        for blob in blobs:
            blob = np.array(blob)
            x_min, x_max = np.min(blob[:, 0]).item(), np.max(blob[:, 0]).item()
            y_min, y_max = np.min(blob[:, 1]).item(), np.max(blob[:, 1]).item()
            bounds.append([(x_min, y_min),(x_max, y_max)])
        return bounds

    def get_display_coords(self, x, y, display=(360, 360), world=(30, 15)):
        x = (display[0]*0.5) - (x * (display[0]/world[0]))
        y = display[1] - ((display[1]*0.5) - (y * (display[1]/world[1])))
        x, y = np.clip(x, 0, display[0]-1), np.clip(y, 0, display[1]-1)
        return int(x), int(y)

    def get_obstacle_bound(self, image, tol=100):
        rectangle_bounds = self.get_rectangles(image, tol)
        return_bounds = rectangle_bounds

        parent_idx = []
        for i in range(len(return_bounds)):
            for j in range(len(return_bounds)):
                rect1, rect2 = return_bounds[i], return_bounds[j]
                if (i != j and not self.contains_rectangle(rect1, rect2)):
                    parent_idx.append(i)
        return_bounds = np.array(return_bounds)[parent_idx]
        return return_bounds