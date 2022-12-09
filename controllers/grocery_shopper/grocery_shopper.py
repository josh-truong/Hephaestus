"""grocery controller."""
import math
import numpy as np
import py_trees
import matplotlib.pyplot as plt

from behaviors.models import DisplayOverlays
from behaviors.models import Localization
from behaviors.models import Planning
from behaviors.models import ConfigSpace
from behaviors import Blackboard
from behaviors import Controller
from behaviors import Mapping
from behaviors import FilteringMap
from behaviors import LineDetection
from behaviors import MapBounds
from behaviors import CameraBounds
from behaviors import ObstacleAvoidance
from behaviors import Planning
from behaviors import RRT

blackboard = Blackboard()
writer, reader = blackboard.get()


#Initialization
print("=== Initializing Grocery Shopper...")
robot = reader.robot.robot
robot.step(int(robot.getBasicTimeStep()))
writer.robot.pose = Localization(writer, reader).get_pose()
pose = reader.robot.pose
planning = Planning()

x, y = DisplayOverlays(writer, reader).get_display_coords(pose.x, pose.y)
nodes = planning.rrt(
    starting_point=(x, y), 
    goal_point=np.random.randint(0,360,2), 
    k=1000, 
    delta_q=10, 
    map=ConfigSpace().run(reader.env.map.map))
writer.env.waypoints = planning.getWaypoints(nodes)

autonomous_mapping = py_trees.composites.Sequence("Sequence")
autonomous_mapping.add_child(Controller(name="Controlling Robot", writer=writer, reader=reader))
autonomous_mapping.add_child(ObstacleAvoidance(name="Avoiding Obstacle", writer=writer, reader=reader))
autonomous_mapping.add_child(RRT(name="RRT", writer=writer, reader=reader))
autonomous_mapping.add_child(Mapping(name="Mapping Controller", writer=writer, reader=reader))
autonomous_mapping.add_child(FilteringMap(name="Filtering Controller", writer=writer, reader=reader))
# autonomous_mapping.add_child(CameraBounds(name="Detecting Cube", writer=writer, reader=reader))
autonomous_mapping.setup_with_descendants()

# path_planning = py_trees.composites.Sequence("Sequence")
# path_planning.add_child(MapBounds(name="Detecting Obstacle Bounds", writer=writer, reader=reader))
# autonomous_mapping.add_child(LineDetection(name="Line Detection", writer=writer, reader=reader))
# path_planning.add_child(RRT(name="Running RRT", writer=writer, reader=reader))

block_collection = py_trees.composites.Sequence("Sequence")
block_collection.setup_with_descendants()


counter = 0
# Main Loop
while robot.step(int(robot.getBasicTimeStep())) != -1:
    if (reader.env.behavior_state == 0):
        if (reader.env.num_completed_paths == reader.env.max_completed_paths): writer.env.behavior_state = 1
        autonomous_mapping.tick_once()
    elif (reader.env.behavior_state == 1):
        path_planning.tick_once()
    else:
        pass
