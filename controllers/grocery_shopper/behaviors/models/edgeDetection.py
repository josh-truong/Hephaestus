"""
edge_detection.py

Created on Sat Dec 3 2022
@Lead: Joshua Truong

Did I make my own rectangle detection algorithm?
"""
import numpy as np
from collections import defaultdict
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

class EdgeDetection():
    def __init__(self, m, frequency=50):
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
        # self.m.Mapping.get_map_variance()

    def get_rectangles(self, map, display=False):
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
                coordinate_list.extend([(y-1, x),(y+1, x),(y, x-1),(y, x+1)])
            return np.asarray(coordinates_in_blob)

        def get_blobs(img_mask):
            img_mask_height, img_mask_width = img_mask.shape[0], img_mask.shape[1]
            blobs_list = []
            img_mask_copy = img_mask.copy()
            for idx in range(img_mask_height*img_mask_width):
                y, x = idx // img_mask_width, idx % img_mask_width
                if (img_mask[y][x] > 0):
                    blob_coords = expand(img_mask_copy, (y, x))
                    if (len(blob_coords)):
                        blobs_list.append(blob_coords)
            return np.asarray(blobs_list, dtype=object)

        def get_rectangle_bounds(blobs):
            # Find bounds of blobs
            rectangle_bounds = []
            for blob in blobs:
                blob = np.array(blob)
                x_min, x_max = np.min(blob[:, 0]), np.max(blob[:, 0])
                y_min, y_max = np.min(blob[:, 1]), np.max(blob[:, 1])
                rectangle_bounds.append([(x_min, y_min),(x_max, y_max)])
            return rectangle_bounds

        def rebound_residuals(blobs, bounds):
            """Identify misclassified points and create a new rectangle bound"""
            def residual(bound, pt):
                pt_x, pt_y = pt[:,0], pt[:,1]
                (X1, Y1), (X2, Y2) = bound
                
                x_min, y_min = min(X1,X2), min(Y1,Y2)
                x_max, y_max = max(X1,X2), max(Y1,Y2)

                d_top = abs(y_max - pt_y)
                d_bottom = abs(y_min - pt_y)
                corner_y = np.zeros(len(pt))
                corner_y[d_top < d_bottom] = y_max
                corner_y[d_top >= d_bottom] = y_min


                d_left = abs(x_min - pt_x)
                d_right = abs(x_max - pt_x)
                corner_x = np.zeros(len(pt))
                corner_x[d_left < d_right] = x_min
                corner_x[d_left >= d_right] = x_max

                d_cx = corner_x - pt_x
                d_cy = corner_y - pt_y
                d_corner = np.sqrt(d_cx*d_cx + d_cy*d_cy)
                return np.min([d_top, d_bottom, d_left, d_right, d_corner], axis=0)

            blobs = np.array(blobs)
            return_bounds = []
            for i, blob in enumerate(blobs):
                return_bounds.append(bounds[i])
                rm_idx = np.where(residual(bounds[i], blobs[i]) > 5)
                unclustered_blob = blobs[i][rm_idx]
                new_map = np.zeros(map.shape)
                new_map[unclustered_blob[:,1],unclustered_blob[:,0]] = 1
                new_blobs = get_blobs(new_map)
                new_blobs_size = np.array([len(blob) for blob in new_blobs])
                new_bounds = get_rectangle_bounds(new_blobs)
                if (len(new_bounds) != 0):
                    return_bounds.extend(new_bounds)
            return return_bounds


        # Generate blobs
        blobs = get_blobs(map)
        blobs_size = np.array([len(blob) for blob in blobs])
        blobs = blobs[np.where(blobs_size > 100)]
        rectangle_bounds = get_rectangle_bounds(blobs)
        rectangle_bounds = rebound_residuals(blobs, rectangle_bounds)


        if (display):
            fig, ax = plt.subplots()
            ax.imshow(map)
            for blob in blobs:
                blob = np.array(blob)
                ax.scatter(blob[:,0],blob[:,1], s=0.5)
                ax.scatter(180, 180, s=1)
            for i, bounds in enumerate(rectangle_bounds):
                X1,Y1  = bounds[0]
                X2, Y2 = bounds[1]
                currentAxis = plt.gca()
                bounding_box = Rectangle((min(X1,X2), min(Y1,Y2)), abs(X2-X1), abs(Y2-Y1), fill=None,  color='red', lw=2)
                currentAxis.add_patch(bounding_box)
            plt.show()
        return rectangle_bounds

    def get_obstacle_bound(self, display=False):
        def contains_rectangle(rect1, rect2):
            rect1_x1, rect1_y1 = rect1[0]
            rect1_x2, rect1_y2 = rect1[1]
            rect2_x1, rect2_y1 = rect2[0]
            rect2_x2, rect2_y2 = rect2[1]
            return rect1_x1 < rect2_x1 < rect2_x2 < rect1_x2 and rect1_y1 < rect2_y1 < rect2_y2 < rect1_y2

        map = self.m.Mapping.Map.map
        rectangle_bounds = self.get_rectangles(map, display=False)
        pose = self.m.Localization.Pose
        x, y = self.m.Mapping.get_display_coords(pose.x, pose.y)

        return_bounds = []
        for bound in rectangle_bounds:
            (X1, Y1), (X2, Y2) = bound
            x_min, y_min = min(X1,X2), min(Y1,Y2)
            x_max, y_max = max(X1,X2), max(Y1,Y2)

            if (x >= x_min and x <= x_max and y >= y_min and y <= y_max):
                continue
            return_bounds.append(bound)

        parent_idx = []
        for i in range(len(return_bounds)):
            for j in range(len(return_bounds)):
                rect1, rect2 = return_bounds[i], return_bounds[j]
                if (i != j and not contains_rectangle(rect1, rect2)):
                    parent_idx.append(i)
        return_bounds = np.array(return_bounds)[parent_idx]

        # if (display):
        plt.imshow(map)
        for i, bounds in enumerate(return_bounds):
            X1,Y1  = bounds[0]
            X2, Y2 = bounds[1]
            currentAxis = plt.gca()
            bounding_box = Rectangle((min(X1,X2), min(Y1,Y2)), abs(X2-X1), abs(Y2-Y1), fill=None,  color='red', lw=2)
            currentAxis.add_patch(bounding_box)
        plt.pause(3)
        plt.close()
        return return_bounds

