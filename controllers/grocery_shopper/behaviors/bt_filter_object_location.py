import py_trees
import numpy as np
import matplotlib.pyplot as plt
from .models import DisplayOverlays

"""
Filters Object Location in order to remove any outliers.
- For example estimated location must be within the shelf.
Therefore, if its on the aisle then it is invalid.
"""
class LocationFilter(py_trees.behaviour.Behaviour):
    def __init__(self, name, writer, reader):
        super(LocationFilter, self).__init__(name)
        # self.logger.debug("%s [%s::__init__()]" % (self.name, self.__class__.__name__))
        self.w, self.r = writer, reader

    def log_message(self, function_name: str, feedback_message=""):
        self.logger.debug("%s [%s::%s][%s]" % (self.name, function_name, self.__class__.__name__, feedback_message))

    def setup(self):
        # self.log_message("setup()")
        self.display = DisplayOverlays(self.w, self.r)

    def initialise(self):
        # self.log_message("initialise()")
        pass

    def update(self):
        map = self.r.env.map.map
        object_location = self.r.env.object_location
        new_object_location = []
        for x,y,depth in object_location:
            x, y = self.display.get_display_coords(x, y)
            if (map[y][x] == 1):
                x, y = self.display.get_world_coords(x, y)
                new_object_location.append([x,y,depth])
        self.w.env.object_location = new_object_location
        self.log_message("update()", f"Filtered {len(object_location) - len(new_object_location)} outliers")
        return py_trees.common.Status.SUCCESS
        
    def terminate(self, new_status):
        self.log_message("terminate()", "%s->%s" % (self.status, new_status))
