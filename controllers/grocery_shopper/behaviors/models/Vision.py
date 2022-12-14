"""
Vision.py

Last updated on Fri Dec 9 2022
@Lead: Hailey Kellackey
"""

from controller import Robot
import math
import numpy as np
from matplotlib import pyplot as plt

import copy
from controller import Camera

#***************************USING VISION****************************

#call vision.detect() and pass it a true if you want it to print the mask to the screen
class Vision:
    def __init__(self, width, height):
        self.img_width = width
        self.img_height = height
        self.color_ranges = []
        self.color_ranges.append(((0, 200,200), (7,240,240)))
        return
        
    #runs the whole vision protocall returns tuple: (boolean indicating if a blob was found, image, mask showing location of the blob)
    def detect(self, img, toggleShow = False):
        img_mask = self.do_color_filtering(img)
        blobs = []
        blobs = self.get_blobs(img_mask)
        if len(blobs) != 0:
            object_positions_list = self.get_blob_centroids(blobs)
            isObj = True
        else :
            isObj =  False
       
        if toggleShow:
            self.show_image(img, img_mask, isObj)
        return self.get_blob_centroids(blobs), blobs, img_mask
 
    #prints the image
    def show_image(self, img, mask, isObj):
        if isObj:
            fig, axs = plt.subplots(1,2)
            fig.suptitle('Blob Detected')
            img = np.array(img)
            img = img.transpose(1, 0, 2)
            axs[0].imshow(img)
            axs[1].imshow(mask)
            plt.tight_layout()
            plt.show()
        return
        
    def check_if_color_in_range(self, bgr_tuple):    
        for entry in self.color_ranges:
            lower, upper = entry[0], entry[1]
            in_range = True
            for i in range(len(bgr_tuple)):
                if bgr_tuple[i] < lower[i] or bgr_tuple[i] > upper[i]:
                    in_range = False
                    break
            if in_range: return True
        return False

    #purpose: filters the image to find any occurances of the goal colors
    #params: img-an array of bgr tuples
    #return: mask-an array of 0s and 1s with ones indicating instances f goal color(s)
    def do_color_filtering(self, img):
        mask = np.zeros([self.img_height, self.img_width]) 
        
        for i in range(self.img_width*self.img_height):
            x, y = i%self.img_width, i//self.img_width
            red   = img[x][y][0]
            green = img[x][y][1]
            blue  = img[x][y][2]
            if self.check_if_color_in_range((blue, green, red)):
                mask[y,x] = 1
            else:
                mask[y,x] = 0
        return mask
    
    
    #purpose: groups all occurances of goal color touching the current pixel into blobs
    #params: img_mask- binary mask of an image, cur_coordinate
    #return: none, alters list:coordinates_in_blob 
    def expand(self, img_mask, cur_coordinate, coordinates_in_blob):
        if cur_coordinate[0] < 0 or cur_coordinate[1] < 0 or cur_coordinate[0] >= img_mask.shape[0] or cur_coordinate[1] >= img_mask.shape[1]: 
            return
        if img_mask[cur_coordinate[0], cur_coordinate[1]] == 0.0: 
            return
        img_mask[cur_coordinate[0],cur_coordinate[1]] = 0
        coordinates_in_blob.append(cur_coordinate)
    
        above = [cur_coordinate[0]-1, cur_coordinate[1]]
        below = [cur_coordinate[0]+1, cur_coordinate[1]]
        left = [cur_coordinate[0], cur_coordinate[1]-1]
        right = [cur_coordinate[0], cur_coordinate[1]+1]
        for coord in [above, below, left, right]: 
            self.expand(img_mask, coord, coordinates_in_blob)
    
   
    #purpose: Take a mask image as input, group each blob of non-zero pixels as a detected object, 
      #            and return a list of lists containing the coordinates of each pixel belonging to each blob.
    #params: img-an array of bgr tuples
    #return: list of all blobs
    def get_blobs(self, img_mask):
      img_mask_height = img_mask.shape[0]
      img_mask_width = img_mask.shape[1]
     
      mask = copy.deepcopy(img_mask)
      blobs_list = [] 
      
      for y in range(img_mask_height):
        for x in range(img_mask_width):
          if mask[y,x] == 1:
            coords = []
            self.expand(mask, (y,x), coords) 
            blobs_list.append(coords)
      return blobs_list
          
    def get_blob_centroids(self, blobs_list):
        object_positions_list = []
        for blob in blobs_list:
            center = np.mean(blob, axis=0)
            object_positions_list.append((center[0], center[1]))
        return object_positions_list
