import py_trees
import numpy as np
from scipy.signal import convolve2d
from .models import DisplayOverlays
from .models import ConfigSpace

class FilteringMap(py_trees.behaviour.Behaviour):
    def __init__(self, name, writer, reader):
        super(FilteringMap, self).__init__(name)
        # self.logger.debug("%s [%s::__init__()]" % (self.name, self.__class__.__name__))
        self.w, self.r = writer, reader

    def log_message(self, function_name: str, feedback_message=""):
        self.logger.debug("%s [%s::%s][%s]" % (self.name, function_name, self.__class__.__name__, feedback_message))

    def setup(self):
        # self.log_message("setup()")
        
        self.counter = 0
        self.frequency = self.r.env.refresh_hz*2
        self.display = DisplayOverlays(self.w, self.r)
        self.configSpace = ConfigSpace()

    def initialise(self):
        # self.log_message("initialise()")
        pass

    def update(self):
        self.counter += 1
        self.feedback_message = f"Convolving Map in {self.frequency - self.counter}"
        if (self.counter%self.frequency == 0):
            self.w.robot.msg = "Denoising map."
            self.counter = 0

            map = self.configSpace.filter_by_convolving(self.r.env.map.map, self.r.env.ftol)
            self.w.env.map.map = map
            self.display.redraw_display(map)
            self.display.draw_waypoints()
            
        self.log_message("update()", self.feedback_message)
        return py_trees.common.Status.SUCCESS
        
    def terminate(self, new_status):
        # self.log_message("terminate()", "%s->%s" % (self.status, new_status))
        pass