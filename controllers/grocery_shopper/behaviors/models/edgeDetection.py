"""
edge_detection.py

Created on Sat Dec 3 2022
@Lead: Joshua Truong

Did I make my own rectangle detection algorithm?
"""
import numpy as np
from collections import defaultdict
from matplotlib.patches import Rectangle

class EdgeDetection():
    def __init__(self, writer, reader):
        self.w, self.r = writer, reader

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
    
    # def residual(bound, pt):
    #     """Identify misclassified points and create a new rectangle bound"""
    #     pt = np.array(pt)
    #     pt_x, pt_y = pt[:,0], pt[:,1]
    #     (X1, Y1), (X2, Y2) = bound
        
    #     x_min, y_min = min(X1,X2), min(Y1,Y2)
    #     x_max, y_max = max(X1,X2), max(Y1,Y2)

    #     d_top = abs(y_max - pt_y)
    #     d_bottom = abs(y_min - pt_y)
    #     corner_y = np.zeros(len(pt))
    #     corner_y[d_top < d_bottom] = y_max
    #     corner_y[d_top >= d_bottom] = y_min

    #     d_left = abs(x_min - pt_x)
    #     d_right = abs(x_max - pt_x)
    #     corner_x = np.zeros(len(pt))
    #     corner_x[d_left < d_right] = x_min
    #     corner_x[d_left >= d_right] = x_max

    #     d_cx = corner_x - pt_x
    #     d_cy = corner_y - pt_y
    #     d_corner = np.sqrt((d_cx**2).astype(int) + (d_cy**2).astype(int))
    #     return np.min([d_top, d_bottom, d_left, d_right, d_corner], axis=0).tolist()


    # def get_rectangles(self, image, tol):
    #     blobs = np.array(blobs)
    #     return_bounds = []
    #     for i, blob in enumerate(blobs):
    #         return_bounds.append(bounds[i])
    #         rm_idx = np.where(np.array(residual(bounds[i], blobs[i])) > 5)
    #         unclustered_blob = blobs[i][rm_idx]
    #         if (len(unclustered_blob) == 0): continue
    #         new_image = np.zeros(image.shape)
    #         new_image[unclustered_blob[:,1],unclustered_blob[:,0]] = 1
    #         new_blobs = get_blobs(new_image)
    #         new_blobs_size = np.array([len(blob) for blob in new_blobs])
    #         new_bounds = get_rectangle_bounds(new_blobs)
    #         if (len(new_bounds) != 0):
    #             return_bounds.extend(new_bounds)
    #     return return_bounds

    #     # Generate blobs
    #     blobs = get_blobs(image)
    #     blobs_size = np.array([len(blob) for blob in blobs])
    #     blobs = blobs[np.where(blobs_size >= tol)]
    #     rectangle_bounds = get_rectangle_bounds(blobs)
    #     rectangle_bounds = rebound_residuals(blobs, rectangle_bounds)
    #     return rectangle_bounds

    def get_display_coords(self, x, y, display=(360, 360), world=(30, 15)):
        x = (display[0]*0.5) - (x * (display[0]/world[0]))
        y = display[1] - ((display[1]*0.5) - (y * (display[1]/world[1])))
        x, y = np.clip(x, 0, display[0]-1), np.clip(y, 0, display[1]-1)
        return int(x), int(y)

    def contains_rectangle(self, rect1, rect2):
            rect1_x1, rect1_y1 = rect1[0]
            rect1_x2, rect1_y2 = rect1[1]
            rect2_x1, rect2_y1 = rect2[0]
            rect2_x2, rect2_y2 = rect2[1]
            return rect1_x1 < rect2_x1 < rect2_x2 < rect1_x2 and rect1_y1 < rect2_y1 < rect2_y2 < rect1_y2

    def get_obstacle_bound(self, image, tol=100):
        rectangle_bounds = self.get_rectangles(image, tol)
        return_bounds = rectangle_bounds

        # return_bounds = []
        # for bound in rectangle_bounds:
        #     (X1, Y1), (X2, Y2) = bound
        #     x_min, y_min = min(X1,X2), min(Y1,Y2)
        #     x_max, y_max = max(X1,X2), max(Y1,Y2)

        #     # Detects if robot is in boundary, which makes boundary invalid
        #     if (x >= x_min and x <= x_max and y >= y_min and y <= y_max):
        #         continue
        #     return_bounds.append(bound)

        parent_idx = []
        for i in range(len(return_bounds)):
            for j in range(len(return_bounds)):
                rect1, rect2 = return_bounds[i], return_bounds[j]
                if (i != j and not self.contains_rectangle(rect1, rect2)):
                    parent_idx.append(i)
        return_bounds = np.array(return_bounds)[parent_idx]
        return return_bounds

    def draw_bounds(self, bounds, color=0xFF0000):
        display = self.w.device.display
        map = self.r.env.map.map

        width, height = map.shape
        map = (map.T*255).astype(int)
        map = np.dstack([map, map, map])
        display = self.r.device.display
        ir = display.imageNew(map.tolist(), display.RGB, width, height)
        display.imagePaste(ir, 0, 0, False)
        display.imageDelete(ir)

        display.setColor(color)
        for bound in bounds:
            (X1, Y1), (X2, Y2) = bound.tolist()
            x_min, y_min = min(X1,X2), min(Y1,Y2)
            x_max, y_max = max(X1,X2), max(Y1,Y2)
            display.drawRectangle(min(X1,X2), min(Y1,Y2), abs(X2-X1), abs(Y2-Y1))
