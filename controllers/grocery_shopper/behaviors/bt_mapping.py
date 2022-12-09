"""
bt_mapping.py

Last updated on Fri Dec 9 2022
@Lead: Joshua Truong
"""

import py_trees
import numpy as np
from .models import MappingModel
from .models import DisplayOverlays

class Mapping(py_trees.behaviour.Behaviour):
    def __init__(self, name, writer, reader):
        super(Mapping, self).__init__(name)
        # self.logger.debug("%s [%s::__init__()]" % (self.name, self.__class__.__name__))
        self.w, self.r = writer, reader

    def log_message(self, function_name: str, feedback_message=""):
        self.logger.debug("%s [%s::%s][%s]" % (self.name, function_name, self.__class__.__name__, feedback_message))

    def setup(self):
        # self.log_message("setup()")
        self.MapModel = MappingModel(self.w, self.r)
        self.Display = DisplayOverlays(self.w, self.r)

    def initialise(self):
        # self.log_message("initialise()")
        pass

    def update(self):
        if (self.r.device.disable_lidar): return py_trees.common.Status.SUCCESS
        pose = self.r.robot.pose
        point_cloud = self.MapModel.get_lidar_point_cloud(pose)
        self.Display.display_point_cloud(point_cloud)
        self.Display.draw_robot_position()

        self.feedback_message = f"{len(point_cloud)} lidar points detected."
        self.log_message("update()", self.feedback_message)

        return py_trees.common.Status.SUCCESS
        
    def terminate(self, new_status):
        # self.log_message("terminate()", "%s->%s" % (self.status, new_status))
        pass