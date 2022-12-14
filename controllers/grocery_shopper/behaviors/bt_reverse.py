"""
bt_reverse.py

Last updated on Fri Dec 9 2022
@Lead: Joshua Truong
"""

import py_trees
import numpy as np
from .models import SpeedController

class Reverse(py_trees.behaviour.Behaviour):
    """
    Reverse until distance reached
    """
    def __init__(self, name, writer, reader):
        super(Reverse, self).__init__(name)
        # self.logger.debug("%s [%s::__init__()]" % (self.name, self.__class__.__name__))
        self.w, self.r = writer, reader

    def log_message(self, function_name: str, feedback_message=""):
        self.logger.debug("%s [%s::%s][%s]" % (self.name, function_name, self.__class__.__name__, feedback_message))

    def setup(self):
        # self.log_message("setup()")
        self.Driver = SpeedController(self.w, self.r)
        self.detect_depth  = 0.7
        self.desired_depth = 1.5
        self.current_depth = self.detect_depth
        self.switch = False

    def initialise(self):
        # self.log_message("initialise()")
        pass

    def update(self):
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
            self.w.robot.msg = "Going backwards"
            self.current_depth = self.desired_depth
            self.switch = True
            self.log_message("update()", f"Nearest object: {mean_depth:.3f} m.")
            speed = self.r.constants.robot.MAX_SPEED
            self.Driver.set_wheel_joint_vel(-speed, -speed)
            self.Driver.localization.update_odometry()
            return py_trees.common.Status.RUNNING
        elif (self.switch):
            self.w.robot.msg = "Calling RRT to run."
            self.w.env.rerun_rrt = self.switch
            self.switch = False
        else:
            self.log_message("update()", f"Tiago has finished backing! Rerunning RRT.")
            self.current_depth = self.detect_depth
            return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        # self.log_message("terminate()", "%s->%s" % (self.status, new_status))
        pass
