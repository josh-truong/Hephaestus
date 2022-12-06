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

blackboard = Blackboard()
writer, reader = blackboard.get()


#Initialization
print("=== Initializing Grocery Shopper...")
robot = reader.robot.robot

root = py_trees.composites.Sequence("Sequence")
root.add_child(Controller(name="Controlling Robot", writer=writer, reader=reader))
root.add_child(Mapping(name="Mapping Controller", writer=writer, reader=reader))
root.add_child(FilteringMap(name="Filtering Controller", writer=writer, reader=reader))
root.add_child(MapBounds(name="Detecting Obstacle Bounds", writer=writer, reader=reader))
root.add_child(CameraBounds(name="Detecting Cube", writer=writer, reader=reader))
root.setup_with_descendants()


def get_display_coords(x, y, display=(360, 360), world=(30, 15)):
    x = (display[0]*0.5) - (x * (display[0]/world[0]))
    y = display[1] - ((display[1]*0.5) - (y * (display[1]/world[1])))
    x, y = np.clip(x, 0, display[0]-1), np.clip(y, 0, display[1]-1)
    return [int(x), int(y)]

counter = 0
# Main Loop
while robot.step(int(robot.getBasicTimeStep())) != -1:
    root.tick_once()

    counter+=1
    if (counter%300==0):
        fig, ax = plt.subplots()
        ax.imshow(np.load('C:\\Users\\joshk\\OneDrive\\Desktop\\CSCI 3302 - Intro to Robotics\\Hephaestus\\controllers\\grocery_shopper\\assets\\map.npy'))
        object_location = np.array(reader.env.object_location)
        object_location = np.array([get_display_coords(x, y) for x,y,_ in object_location])
        ax.scatter(object_location[:,0],object_location[:,1],s=2,color='red')
        plt.pause(3)
        plt.close()
    

