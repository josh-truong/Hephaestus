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
from behaviors import DenoiseMap
from behaviors import LocationFilter
from behaviors import LineDetection
from behaviors import MapBounds
from behaviors import CameraVision
from behaviors import ObstacleAvoidance
from behaviors import Planning
from behaviors import RRT
from behaviors import KMeans
from behaviors import LockAndLoad

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
    map=ConfigSpace().convolve(reader.env.map.map))
writer.env.waypoints = planning.getWaypoints(nodes)

"""
Main method of autonomous mapping uses RRT to explore its environment and to allow
the robot to update the map using it's lidar readings. Ideally the robot will call
rrt several times at the beginning but as it learns about the environment rrt will 
be called less.

Therefore, the ObstacleAvoidance behavior is crucial for the overal performance as it 
will call RRT if theres a conflict with the robot path. ObstacleAvoidance implements 
both a passive and active avoidance behavior.
    1) Actively check the waypoints with the current map and see if any collision
    between waypoint and obstacle has occured and call to rerun rrt.
    2) Implementing a state behavior with lidar readings if using the 25th percentile
    of the lidar readings to determine the mean distance of the closest obstacle. If a 
    threshold has been reached tell the robot to reverse to a certain distance and rerun
    rrt.

The CameraVision goal is to detect the objects of interests estimate the distance
of the object and calculate an estimate location of the object in world coordinates.
Save the location in object_location variable. 
"""
autonomous_mapping = py_trees.composites.Sequence("Sequence")
autonomous_mapping.add_child(Controller(name="Controlling Robot", writer=writer, reader=reader))
autonomous_mapping.add_child(ObstacleAvoidance(name="Avoiding Obstacle", writer=writer, reader=reader))
autonomous_mapping.add_child(RRT(name="RRT", writer=writer, reader=reader))
autonomous_mapping.add_child(Mapping(name="Mapping Controller", writer=writer, reader=reader))
autonomous_mapping.add_child(DenoiseMap(name="Denoising Map", writer=writer, reader=reader))
# autonomous_mapping.add_child(CameraVision(name="Detecting Cube", writer=writer, reader=reader))
autonomous_mapping.setup_with_descendants()


"""
path_planning sequence will run only once.
- Detect the lines/edges of the obstacle and draw on map with a width
The width of the lines will hopefully merge with other nearby lines.
    - The Hough Lines are represented in white pixels
- Draw the bounds of the white pixels
    - Represented by the red rectangles
- Running kmeans of through the estimate object location
    - The k centroids will represent with high probability of the object location.
    - We can then run rrt to the estimate location of the object. And have the camera 
    vision assist in picking the blocks
"""
path_planning = py_trees.composites.Sequence("Sequence")
path_planning.add_child(LineDetection(name="Line Detection", writer=writer, reader=reader))
path_planning.add_child(MapBounds(name="Obstacle Boundings", writer=writer, reader=reader))
path_planning.add_child(LocationFilter(name="Filtering Object Location", writer=writer, reader=reader))
path_planning.add_child(KMeans(name="K-means Clustering", writer=writer, reader=reader))
path_planning.setup_with_descendants()


block_collection = py_trees.composites.Sequence("Sequence")
block_collection.add_child(Controller(name="Controlling Robot", writer=writer, reader=reader))
block_collection.add_child(LockAndLoad(name="LockAndLoad", writer=writer, reader=reader))
block_collection.setup_with_descendants()
writer.env.waypoints = [
            [5.210931040630611, -2.210988145012728],
            [-13.192591374679662, -2.22138003370971],
            [-13.154953213728083,  -5.756066295328318],
            [5.501602885777003,  -5.553924695245729],
            [5.423878744330168, 2.263936779778975],
            [-12.65776213906247, 2.0846689769211477],
            [-12.708079837301646, 5.7219656125502905],
            [5.387549771991973, 5.9776086031667655]]
writer.env.state = 0


counter = 0
# Main Loop
# writer.env.behavior_state = 1
while robot.step(int(robot.getBasicTimeStep())) != -1:
    if (reader.env.behavior_state == 0):
        if (reader.env.num_completed_paths == reader.env.max_completed_paths): writer.env.behavior_state = 1
        autonomous_mapping.tick_once()
    elif (reader.env.behavior_state == 1):
        path_planning.tick_once()
        writer.env.behavior_state = 2
        writer.env.rerun_rrt = True
    elif (reader.env.behavior_state == 2):
        # if (counter%100==0):
        #     DisplayOverlays(writer, reader).draw_estimated_location_on_map(reader.env.kmeans_location, color=0x00FF00)
        block_collection.tick_once()
        pass


    counter += 1
    if (counter%2000 == 0):
        np.save("assets/map.npy", reader.env.map.map)
        np.save("assets/viable_waypoints.npy", reader.env.vaiable_waypoints)
        np.save("assets/object_location.npy", reader.env.object_location)
        writer.robot.msg = "Map, waypoints, est. cube location saved!"