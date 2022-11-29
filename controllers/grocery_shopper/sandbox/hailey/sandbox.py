
from controller import Robot
import math
import numpy as np
from matplotlib import pyplot as plt

#from controller import 
from controller import Camera


color_ranges = []
color_ranges.append((0, 200,200), (7,240,240))


def check_if_color_in_range(bgr_tuple):
    
  #@param bgr_tuple: Tuple of BGR values
  #returns Boolean: True if bgr_tuple is in any of the color ranges specified in color_ranges
  
    global color_ranges
    for entry in color_ranges:
        lower, upper = entry[0], entry[1]
        in_range = True
        for i in range(len(bgr_tuple)):
            if bgr_tuple[i] < lower[i] or bgr_tuple[i] > upper[i]:
                in_range = False
                #print(bgr_tuple)
                break
        if in_range: return True
    return False

def do_color_filtering(img):
  # Color Filtering
  # Objective: Take an RGB image as input, and create a "mask image" to filter out irrelevant pixels
  # Definition "mask image":
  #    An 'image' (really, a matrix) of 0s and 1s, where 0 indicates that the corresponding pixel in 
  #    the RGB image isn't important (i.e., what we consider background) and 1 indicates foreground.
  #
  #    Importantly, we can multiply pixels in an image by those in a mask to 'cancel out' all of the pixels we don't
  #    care about. Once we've done that step, the only non-zero pixels in our image will be foreground
  #
  # Approach:
  # Create a mask image: a matrix of zeros ( using np.zeroes([height width]) ) of the same height and width as the input image.
  # For each pixel in the input image, check if it's within the range of allowable colors for your detector
  #     If it is: set the corresponding entry in your mask to 1
  #     Otherwise: set the corresponding entry in your mask to 0 (or do nothing, since it's initialized to 0)
  # Return the mask image
    img_height = Camera.getHeight()
    img_width = Camera.getWidth()

  # Create a matrix of dimensions [height, width] using numpy
    mask = np.zeros([img_height, img_width]) # Index mask as [height, width] (e.g.,: mask[y,x])
    
    for x in range(img_width):
        for y in range(img_height):
            red   = image[x][y][0]
            green = image[x][y][1]
            blue  = image[x][y][2]
            if check_if_color_in_range((blue, green, red)):
              mask[y,x] = 1
            else:
              mask[y,x] = 0
  # TODO: Iterate through each pixel (x,y) coordinate of the image, 
  #       checking if its color is in a range we've specified using check_if_color_in_range
  # TIP: You'll need to index into ' mask' using (y,x) instead of (x,y) as you may be
  #      more familiar with, due to how the matrices are stored

    return mask

def expand(img_mask, cur_coordinate, coordinates_in_blob):
  # Find all of the non-zero pixels connected to a location

  # If value of img_mask at cur_coordinate is 0, or cur_coordinate is out of bounds (either x,y < 0 or x,y >= width or height of img_mask) return and stop expanding

  # Otherwise, add this to our blob:
  # Set img_mask at cur_coordinate to 0 so we don't double-count this coordinate if we expand back onto it in the future
  # Add cur_coordinate to coordinates_in_blob
  # Call expand on all 4 neighboring coordinates of cur_coordinate (above/below, right/left). Make sure you pass in the same img_mask and coordinates_in_blob objects you were passed so the recursive calls all share the same objects

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
        expand(img_mask, coord, coordinates_in_blob)

def expand_nr(img_mask, cur_coord, coordinates_in_blob):
  # Non-recursive function to find all of the non-zero pixels connected to a location

  # If value of img_mask at cur_coordinate is 0, or cur_coordinate is out of bounds (either x,y < 0 or x,y >= width or height of img_mask) return and stop expanding
  # Otherwise, add this to our blob:
  # Set img_mask at cur_coordinate to 0 so we don't double-count this coordinate if we expand back onto it in the future
  # Add cur_coordinate to coordinates_in_blob
  # Call expand on all 4 neighboring coordinates of cur_coordinate (above/below, right/left). Make sure you pass in the same img_mask and coordinates_in_blob objects you were passed so the recursive calls all share the same objects
    coordinates_in_blob = []
    coordinate_list = [cur_coord] # List of all coordinates to try expanding to
    img_mask_height = img_mask.shape[0]
    img_mask_width = img_mask.shape[1]
    while len(coordinate_list) > 0:
        cur_coordinate = coordinate_list.pop() # Take the first coordinate in the list and perform 'expand' on it
        
        #Check to make sure cur_coordinate is in bounds, otherwise 'continue'
        if cur_coordinate[0] < 0 or cur_coordinate[0] >= img_mask_height:
          continue
        if cur_coordinate[1] < 0 or cur_coordinate[1] >= img_mask_width:
          continue
       
       #Check to see if the value is 0, if so, 'continue'
        if img_mask[cur_coordinate] == 0:            
          continue
        else:
          img_mask[cur_coordinate] = 0                        #Set image mask at this coordinate to 0
          coordinates_in_blob.append(cur_coordinate)          #Add this coordinate to 'coordinates_in_blob'

          #Add all neighboring coordinates (above, below, left, right) to coordinate_list to expand to them
          coordinate_list.append((cur_coordinate[0]+1, cur_coordinate[1]))         #above
          coordinate_list.append((cur_coordinate[0]-1, cur_coordinate[1]))         #below
          coordinate_list.append((cur_coordinate[0], cur_coordinate[1]-1))         #left
          coordinate_list.append((cur_coordinate[0], cur_coordinate[1]+1))         #right
    #print("coors:   ",len(coordinates_in_blob), " and::     ", coordinates_in_blob, "/n")
    return coordinates_in_blob

def get_blobs(img_mask):
  # Blob detection
  # Objective: Take a mask image as input, group each blob of non-zero pixels as a detected object, 
  #            and return a list of lists containing the coordinates of each pixel belonging to each blob.
  # Recommended Approach:
  # Create a copy of the mask image so you can edit it during blob detection
  # Create an empty blobs_list to hold the coordinates of each blob's pixels
  # Iterate through each coordinate in the mask:
  #   If you find an entry that has a non-zero value:
  #     Create an empty list to store the pixel coordinates of the blob
  #     Call the recursive "expand" function on that position, recording coordinates of non-zero pixels connected to it
  #     Add the list of coordinates to your blobs_list variable
  # Return blobs_list

  img_mask_height = img_mask.shape[0]
  img_mask_width = img_mask.shape[1]
 
  # Copy image mask into local variable using copy.copy
  mask = copy.deepcopy(img_mask)
  blobs_list = [] # List of all blobs, each element being a list of coordinates belonging to each blob

  # Iterate through all 'y' coordinates in img_mask
  #    Iterate through all 'x' coordinates in img_mask
  for y in range(img_mask_height):
    for x in range(img_mask_width):
      if mask[y,x] == 1:
        coords = expand_nr(mask, (y,x), []) # If mask value at [y,x] is 1, call expand_nr on copy of image mask and coordinate (y,x), giving a third argument of an empty list to populate with blob_coords.
        blobs_list.append(coords) #Add blob_coords to blobs_list 

  print(len(blobs_list))
  return blobs_list

def get_blob_centroids(blobs_list):
  # Coordinate retrieval
  # Objective: Take a list of blobs' coordinate lists and return the coordinates of each blob's center.
  # Approach:
  #     Create an object_positions_list
  #     For each list in the blobs_list:
  #     Check to see if the list of coordinates is big enough to be an object we're looking for, otherwise continue. (The legos are several hundred pixels or more in size!)
  #     Find the centroid of all coordinates in the coordinates list (hint: np.mean(my_list_var,axis=0) will take the mean)
  #     Add that centroid (x,y) tuple to object_positions_list
  # Return object_positions_list
    object_positions_list = []

  #Implement blob centroid calculation
    for blob in blobs_list:
      if len(blob) > 1500:
        print("hey  ")
        center = np.mean(blob, axis=0)
        print("center = ", center[0])
        #center_x = np.mean(blob, axis=1)
        object_positions_list.append((center[0], center[1]))

    print("obj ", object_positions_list)
    return object_positions_list

def detect():
    global img_height, img_width
  # Read in image using the imread function
    img = Camera.getImage()
  # Examples of adding color ranges: You'll need to find the right ones for isolating the lego blocks!
  
  

  ########## PART 1 ############
  # Create img_mask of all foreground pixels, where foreground is defined as passing the color filter
    img_mask = do_color_filtering(img)

  ########## PART 2 ############
  # Find all the blobs in the img_mask
    blobs = get_blobs(img_mask)

  ########## PART 3 ############
  # Get the centroids of the img_mask blobs
    object_positions_list = get_blob_centroids(blobs)
    print("len::  ", len(object_positions_list))
    isObj = True if len(object_positions_list) > 0 else False
    
    return (isObj, img, img_mask)
 
def show_image():
    isObj, img, mask = detect()
    plt.imshow(img)
    plt.title('View')
    plt.show()
    
    plt.imshow(mask)
    plt.title('Mask')
    plt.show()
    
    if isObj:
        print(objectfound)