import py_trees
import numpy as np
from scipy.signal import convolve2d
from .models import ConfigSpace

class ObstacleAvoidance(py_trees.behaviour.Behaviour):
    def __init__(self, name, writer, reader):
        super(ObstacleAvoidance, self).__init__(name)
        self.logger.debug("%s [%s::__init__()]" % (self.name, self.__class__.__name__))
        self.w, self.r = writer, reader

    def log_message(self, function_name: str, feedback_message=""):
        self.logger.debug("%s [%s::%s][%s]" % (self.name, function_name, self.__class__.__name__, feedback_message))

    def setup(self):
        self.log_message("setup()")
        self.counter = 0
        self.frequency = 200

    def initialise(self):
        self.log_message("initialise()")

    def update(self):
        def get_display_coords(x, y, display=(360, 360), world=(30, 15)):
            x = (display[0]*0.5) - (x * (display[0]/world[0]))
            y = display[1] - ((display[1]*0.5) - (y * (display[1]/world[1])))
            x, y = np.clip(x, 0, display[0]-1), np.clip(y, 0, display[1]-1)
            return [int(x), int(y)]
        def get_lidar_readings():
            lidar = self.r.device.lidar
            lidar_sensor_readings = lidar.getRangeImage()
            lidar_sensor_readings = lidar_sensor_readings[83:len(lidar_sensor_readings)-83]
            return lidar_sensor_readings

        
        lidar_readings = np.array(get_lidar_readings())
        idx = np.argsort(lidar_readings)[:50]
        mean_dist = np.mean(lidar_readings[idx])
        robotPose = self.r.robot.pose
        print([robotPose.x, robotPose.y, robotPose.theta])

        if (mean_dist < 0.4):
            self.feedback_message = "Obstacle Detected!"
            self.w.env.rerun_rrt = True
            self.log_message("update()", self.feedback_message)
            return py_trees.common.Status.SUCCESS

        self.counter += 1
        if (self.r.env.check_state):
            self.w.env.check_state = False
        # elif(self.counter == 100):
        #     self.feedback_message = "Obstacle Detected!"
        #     self.w.env.rerun_rrt = True
        #     self.log_message("update()", self.feedback_message)
        #     return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.SUCCESS


        self.counter = 0
        waypoints = self.r.env.waypoints
        state = self.r.env.state
        state = state if (state == len(waypoints)-1) else state+1
        waypoint = waypoints[state]

        x, y = get_display_coords(waypoint[0], waypoint[1])
        map = ConfigSpace().run(self.r.env.map.map)
        self.feedback_message = ""
        if (map[y][x] == 1):
            self.feedback_message = "Obstacle Detected!"
            self.w.env.rerun_rrt = True
        self.log_message("update()", self.feedback_message)
        return py_trees.common.Status.SUCCESS
        # return py_trees.common.Status.RUNNING
        
    def terminate(self, new_status):
        self.log_message("terminate()", "%s->%s" % (self.status, new_status))