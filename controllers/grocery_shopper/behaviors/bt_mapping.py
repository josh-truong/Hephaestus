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

        for x, y in point_cloud:
            x, y = self.Display.get_display_coords(x, y)
            xmax_bound = self.w.env.xmax_boundary
            ymax_bound = self.w.env.ymax_boundary
            xmax_bound = x if (x > xmax_bound) else xmax_bound
            ymax_bound = y if (y > ymax_bound) else ymax_bound
            self.w.env.xmax_boundary = int(np.ceil(xmax_bound))
            self.w.env.ymax_boundary = int(np.ceil(ymax_bound))

        self.feedback_message = f"{len(point_cloud)} lidar points detected."
        self.log_message("update()", self.feedback_message)

        return py_trees.common.Status.SUCCESS
        
    def terminate(self, new_status):
        # self.log_message("terminate()", "%s->%s" % (self.status, new_status))
        pass