import py_trees
from .models import Planning
import numpy as np
from .models import ConfigSpace
from .models import DisplayOverlays

class RRT(py_trees.behaviour.Behaviour):
    def __init__(self, name, writer, reader):
        super(RRT, self).__init__(name)
        self.logger.debug("%s [%s::__init__()]" % (self.name, self.__class__.__name__))
        self.w, self.r = writer, reader

    def log_message(self, function_name: str, feedback_message=""):
        self.logger.debug("%s [%s::%s][%s]" % (self.name, function_name, self.__class__.__name__, feedback_message))

    def setup(self):
        self.log_message("setup()")
        self.planner = Planning()
        self.display = DisplayOverlays(self.w, self.r)

    def initialise(self):
        # self.log_message("initialise()")
        pass

    def update(self):
        if (not self.r.env.rerun_rrt): return py_trees.common.Status.SUCCESS
        self.w.env.rerun_rrt = False

        pose = self.r.robot.pose
        x, y = self.display.get_display_coords(pose.x, -pose.y)

        map = ConfigSpace().run(self.r.env.map.map)
        nodes = self.planner.rrt([x, y], np.random.randint(0,360,2), 1000, 10, map)
        waypoints = self.planner.getWaypoints(nodes)

        self.w.env.waypoints = waypoints
        self.w.env.state = 0

        self.feedback_message = f"Generated {len(waypoints)} waypoints."
        self.log_message("update()", self.feedback_message)
        return py_trees.common.Status.SUCCESS
        
    def terminate(self, new_status):
        self.log_message("terminate()", "%s->%s" % (self.status, new_status))