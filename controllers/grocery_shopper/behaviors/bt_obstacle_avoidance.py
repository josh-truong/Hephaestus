import py_trees
import numpy as np
from scipy.signal import convolve2d
from .models import ConfigSpace
from .models import DisplayOverlays
import matplotlib.pyplot as plt
from .models import ControllerModel
from .models import Localization

from .bt_rrt import RRT

class ObstacleAvoidance(py_trees.behaviour.Behaviour):
    def __init__(self, name, writer, reader):
        super(ObstacleAvoidance, self).__init__(name)
        # self.logger.debug("%s [%s::__init__()]" % (self.name, self.__class__.__name__))
        self.w, self.r = writer, reader
        # self.reverse_behavior = py_trees.composites.Sequence("Reverse Sequence")
        # self.reverse_behavior.add_child(Reverse("Reversing", self.w, self.r))
        # self.reverse_behavior.add_child(RRT("RRT for reverse", self.w, self.r))
        # self.reverse_behavior.setup_with_descendants()

    def log_message(self, function_name: str, feedback_message=""):
        self.logger.debug("%s [%s::%s][%s]" % (self.name, function_name, self.__class__.__name__, feedback_message))

    def setup(self):
        self.log_message("setup()")
        self.frequency = 200
        self.counter = 0
        self.display = DisplayOverlays(self.w, self.r)
        self.Driver = ControllerModel(self.w, self.r)

        # Tracking New Behavior if triggered
        self.detect_depth  = 0.7
        self.desired_depth = 1.5
        self.current_depth = self.detect_depth
        self.switch = False


    def initialise(self):
        # self.log_message("initialise()")
        pass

    def update(self):
        # ######################################
        # # Continously check a subset of waypoints to determine if path is still valid
        # waypoints = self.r.env.waypoints
        # n = len(waypoints)
        # state, state_step = self.r.env.state, self.r.env.state_step
        # endstate = np.clip(state+state_step*5, 0, n-1)

        # waypoints = [self.display.get_display_coords(pt[0], pt[1]) for pt in waypoints[state:endstate]]
        # map = np.array(self.r.env.map.map)[np.array(waypoints)]
        # if (len(np.where(map == 1)[0]) > 0):
        #     self.w.env.rerun_rrt = True
        #     self.w.robot.msg = "Path conflict."
        #     self.w.env.waypoints = self.r.env.waypoints[np.clip(self.r.env.state-15, 0, n)]
        #     self.w.env.state = 0
        #     return py_trees.common.Status.SUCCESS

        ######################################
        # Continously check the distance of the robot to avoid obstacle collision
        def get_lidar_readings():
            lidar = self.r.device.lidar
            lidar_sensor_readings = lidar.getRangeImage()
            lidar_sensor_readings = lidar_sensor_readings[83:len(lidar_sensor_readings)-83]
            return lidar_sensor_readings
        # Get the 25th percentile of lidar readings
        lidar_readings = np.array(get_lidar_readings())
        lidar_idx = np.where(lidar_readings <= np.percentile(lidar_readings, 25))[0]
        mean_depth = np.mean(lidar_readings[lidar_idx])

        if (mean_depth < self.current_depth):
            self.log_message("update()", f"Nearest object: {mean_depth:.3f} m.")
            self.w.robot.msg = "Going backwards"
            self.current_depth = self.desired_depth
            self.switch = True
            
            speed = self.r.constants.robot.MAX_SPEED
            self.Driver.set_wheel_joint_vel(-speed, -speed)
            self.Driver.localization.update_odometry()
            return py_trees.common.Status.RUNNING
        elif (self.switch):
            self.w.robot.msg = "Calling RRT to run."
            self.w.env.rerun_rrt = self.switch
            self.switch = False
        else:
            self.current_depth = self.detect_depth
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        # self.log_message("terminate()", "%s->%s" % (self.status, new_status))
        pass