"""
bt_controller.py

Last updated on Fri Dec 9 2022
@Lead: Joshua Truong
"""


import py_trees
import math
import numpy as np
from .models import Pose, SpeedController
from .models import Planning
from .models import Localization
from .models import ConfigSpace

class Controller(py_trees.behaviour.Behaviour):
    """
    Controls the speed of the robot based on IK and waypoints generate by RRT.
    """
    def __init__(self, name, writer, reader):
        super(Controller, self).__init__(name)
        # self.logger.debug("%s [%s::__init__()]" % (self.name, self.__class__.__name__))
        self.w, self.r = writer, reader

        
    def log_message(self, function_name: str, feedback_message=""):
        self.logger.debug("%s [%s::%s][%s]" % (self.name, function_name, self.__class__.__name__, feedback_message))

    def setup(self):
        # self.log_message("setup()")
        self.Driver = SpeedController(self.w, self.r)

    def initialise(self):
        # self.log_message("initialise()")
        pass

    def update(self):
        keyboard = self.r.device.keyboard
        key = keyboard.getKey()
        while(keyboard.getKey() != -1): pass
        if (key == ord('R')):
            self.w.env.rerun_rrt = True
        elif (key == ord('F')):
            map = ConfigSpace().filter_small_islands(self.r.env.map.map)
            self.w.env.map.update_map(map)
        elif (key == ord('Q')):
            self.w.env.behavior_state = 1
        elif (key == ord('S')):
            np.save("assets/map.npy", self.r.env.map.map)
            np.save("assets/viable_waypoints.npy", self.r.env.vaiable_waypoints)
            np.save("assets/object_location.npy", self.r.env.object_location)
            print("Map, waypoints, est. cube location saved!")

        vL, vR = 0, 0
        controller_type = self.r.controller_type
        velocity_rate = self.r.robot.velocity_rate
        if (controller_type == 'manual'):
            vL, vR = self.Driver.manual(key)
            vL, vR = vL*velocity_rate, vR*velocity_rate
        elif (controller_type == 'autonomous'):
            vL, vR = self.Driver.autonomous(velocity_rate)

        self.w.device.disable_lidar = (np.sign(vL) != np.sign(vR))

        self.w.robot.vL, self.w.robot.vR = vL, vR
        self.Driver.set_wheel_joint_vel(vL, vR)
        self.Driver.localization.update_odometry()

        self.log_message("update()", f"Completed paths: {self.r.env.num_completed_paths} out of {self.r.env.max_completed_paths}. Current map bound: [{self.w.env.xmax_boundary}, {self.w.env.ymax_boundary}].")
        return py_trees.common.Status.SUCCESS
        
    def terminate(self, new_status):
        # self.log_message("terminate()", "%s->%s" % (self.status, new_status))
        pass