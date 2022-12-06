"""grocery controller."""
import math
import numpy as np
import py_trees
import matplotlib.pyplot as plt

from behaviors import Blackboard
from behaviors import Controller
from behaviors import Mapping
from behaviors import FilteringMap
from behaviors import MapBounds
from behaviors import CameraBounds
from behaviors import ObstacleAvoidance
from behaviors import RRT

blackboard = Blackboard()
writer, reader = blackboard.get()


#Initialization
print("=== Initializing Grocery Shopper...")
robot = reader.robot.robot

autonomous_mapping = py_trees.composites.Sequence("Sequence")
autonomous_mapping.add_child(Controller(name="Controlling Robot", writer=writer, reader=reader))
autonomous_mapping.add_child(ObstacleAvoidance(name="Avoiding Obstacle", writer=writer, reader=reader))
autonomous_mapping.add_child(RRT(name="RRT", writer=writer, reader=reader))
autonomous_mapping.add_child(Mapping(name="Mapping Controller", writer=writer, reader=reader))
autonomous_mapping.add_child(FilteringMap(name="Filtering Controller", writer=writer, reader=reader))
autonomous_mapping.add_child(CameraBounds(name="Detecting Cube", writer=writer, reader=reader))
autonomous_mapping.setup_with_descendants()

path_planning = py_trees.composites.Sequence("Sequence")
path_planning.add_child(MapBounds(name="Detecting Obstacle Bounds", writer=writer, reader=reader))
path_planning.add_child(RRT(name="Running RRT", writer=writer, reader=reader))


def get_display_coords(x, y, display=(360, 360), world=(30, 15)):
    x = (display[0]*0.5) - (x * (display[0]/world[0]))
    y = display[1] - ((display[1]*0.5) - (y * (display[1]/world[1])))
    x, y = np.clip(x, 0, display[0]-1), np.clip(y, 0, display[1]-1)
    return [int(x), int(y)]

counter = 0
# Main Loop
while robot.step(int(robot.getBasicTimeStep())) != -1:
    if (reader.env.behavior_state == 0):
        autonomous_mapping.tick_once()
    elif (reader.env.behavior_state == 1):
        path_planning.tick_once()
    else:
        pass

    counter+=1
    if (counter%100==0):
        display = reader.device.display
        object_location = np.array(reader.env.object_location)

        display.setColor(0xFFFF00)
        for x,y,_ in object_location:
            x, y = get_display_coords(x, y)
            display.drawRectangle(x-5, y-5, 10,10)


    # if (counter%1000 == 0):
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
    #             coordinates_in_blob.append((x,y))
    #             coordinate_list.extend([(y-1, x),(y+1, x),(y, x-1),(y, x+1)])
    #         return np.asarray(coordinates_in_blob)

    #     def get_blobs(img_mask):
    #         img_mask_height, img_mask_width = img_mask.shape[0], img_mask.shape[1]
    #         blobs_list = []
    #         img_mask_copy = img_mask.copy()
    #         for idx in range(img_mask_height*img_mask_width):
    #             y, x = idx // img_mask_width, idx % img_mask_width
    #             if (img_mask[y][x] > 0):
    #                 blob_coords = expand(img_mask_copy, (y, x))
    #                 if (len(blob_coords)):
    #                     blobs_list.append(blob_coords)
    #         return blobs_list

    #     blobs = get_blobs(reader.env.map.map)
    #     blobs = sorted(blobs, key=len)
    #     blob_size = [len(blob) for blob in blobs]

    #     q25, q75 = np.percentile(blob_size, [25, 75])
    #     IQR = q75-q25
    #     remove_blobs_idx = np.where(blob_size < IQR)
    #     remove_blobs = np.array(blobs)[remove_blobs_idx]
    #     for blobs in remove_blobs:
    #         for x,y in blobs:
    #             writer.env.map.update_pixel(x,y,0)

    #     print(blob_size)
    #     print(q25, q75)