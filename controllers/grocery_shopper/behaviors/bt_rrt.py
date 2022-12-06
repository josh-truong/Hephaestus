import py_trees
from .models import Planning
import numpy as np
from scipy.signal import convolve2d

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

    def initialise(self):
        self.log_message("initialise()")

    def update(self):
        def convertMapToConfigSpace(map):
            kernel = np.ones((10,10))
            map = convolve2d(map, kernel, mode='same')
            map[map > 0] = 1
            return map

        if (not self.r.env.rerun_rrt): return py_trees.common.Status.SUCCESS
        robotPose = self.r.robot.pose
        print(robotPose)
        randPoint = np.random.randint(0,360,2)
        
        map = convertMapToConfigSpace(self.r.env.map.map)
        nodes = self.planner.rrt([robotPose.x, robotPose.y], randPoint, 1000, 10, map)
        waypoints = self.planner.getWaypoints(nodes)
        self.w.env.waypoints = waypoints
        self.w.env.state = 0
        self.w.env.rerun_rrt = False

        self.feedback_message = "Generated RRT"
        self.log_message("update()", self.feedback_message)
        return py_trees.common.Status.SUCCESS
        # return py_trees.common.Status.RUNNING
        
    def terminate(self, new_status):
        self.log_message("terminate()", "%s->%s" % (self.status, new_status))