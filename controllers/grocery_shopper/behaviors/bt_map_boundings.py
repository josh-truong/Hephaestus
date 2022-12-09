import py_trees
from .models import EdgeDetection


class MapBounds(py_trees.behaviour.Behaviour):
    def __init__(self, name, writer, reader):
        super(MapBounds, self).__init__(name)
        # self.logger.debug("%s [%s::__init__()]" % (self.name, self.__class__.__name__))
        self.w, self.r = writer, reader

    def log_message(self, function_name: str, feedback_message=""):
        # self.logger.debug("%s [%s::%s][%s]" % (self.name, function_name, self.__class__.__name__, feedback_message))
        pass

    def setup(self):
        # self.log_message("setup()")
        self.map_frequency = self.r.env.refresh_hz*4
        self.map_counter = 0
        self.detection = EdgeDetection(self.w, self.r)

    def initialise(self):
        # self.log_message("initialise()")
        pass

    def update(self):
        self.map_counter += 1
        self.feedback_message = f"Map detection[{self.map_frequency - self.map_counter}]."
        if (self.map_counter%self.map_frequency == 0):
            self.map_counter = 0
            map_bounds = self.detection.get_obstacle_bound(self.r.env.map.map, 100)
            self.detection.draw_bounds(map_bounds, 0xFF0000)
        
        self.log_message("update()", self.feedback_message)
        return py_trees.common.Status.SUCCESS
        
    def terminate(self, new_status):
        # self.log_message("terminate()", "%s->%s" % (self.status, new_status))
        pass
