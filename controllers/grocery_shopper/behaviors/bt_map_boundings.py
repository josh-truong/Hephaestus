"""
bt_map_boundings.py

Last updated on Fri Dec 9 2022
@Lead: Joshua Truong
"""

import py_trees
from .models import ObjectBound
from .models import DisplayOverlays
import numpy as np
import matplotlib.pyplot as plt


class MapBounds(py_trees.behaviour.Behaviour):
    """
    Calculate the rectangle bounds of the obstacle to determine where obstacle is and generate waypoints in
    between the obstacles to generate the waypoints for the robot to follow in between without colliding.
    The waypoints generate are equal distance from box1 and box2.
    """
    def __init__(self, name, writer, reader):
        super(MapBounds, self).__init__(name)
        # self.logger.debug("%s [%s::__init__()]" % (self.name, self.__class__.__name__))
        self.w, self.r = writer, reader

    def log_message(self, function_name: str, feedback_message=""):
        # self.logger.debug("%s [%s::%s][%s]" % (self.name, function_name, self.__class__.__name__, feedback_message))
        pass

    def setup(self):
        # self.log_message("setup()")
        self.boundMap = ObjectBound()
        self.Displays = DisplayOverlays(self.w, self.r)

    def initialise(self):
        # self.log_message("initialise()")
        pass

    def update(self):
        self.log_message("update()", f"Detecting the bounds of obstacles.")
        blobs = self.boundMap.return_blobs(self.r.env.map.map.copy())
        bounds = self.boundMap.return_bounds(blobs)
        self.Displays.draw_obstacle_bounds(self.w.device.display, bounds, 0xFF0000)
        self.Displays.draw_viable_waypoints()
        plt.imshow(self.r.env.map.map)
        plt.show()
        return py_trees.common.Status.SUCCESS
        
    def terminate(self, new_status):
        # self.log_message("terminate()", "%s->%s" % (self.status, new_status))
        pass
