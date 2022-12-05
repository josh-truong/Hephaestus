"""grocery controller."""
import math
import numpy as np
import py_trees

from behaviors import Blackboard
from behaviors import Controller
from behaviors import Mapping
from behaviors import FilteringMap

blackboard = Blackboard()
writer, reader = blackboard.get()




#Initialization
print("=== Initializing Grocery Shopper...")
py_trees.logging.level = py_trees.logging.Level.DEBUG

robot = reader.robot.robot


root = py_trees.composites.Sequence("Sequence")
root.add_child(Controller(name="Controlling Robot", writer=writer, reader=reader))
root.add_child(Mapping(name="Mapping Controller", writer=writer, reader=reader))
root.add_child(FilteringMap(name="Filtering Controller", writer=writer, reader=reader))
root.setup_with_descendants()

# Main Loop
while robot.step(int(robot.getBasicTimeStep())) != -1:
    root.tick_once()