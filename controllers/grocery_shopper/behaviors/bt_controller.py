import py_trees
import math
import numpy as np
from .models import Pose, ControllerModel
from .models import Planning
from scipy.signal import convolve2d
from .models import Localization

class Controller(py_trees.behaviour.Behaviour):
    def __init__(self, name, writer, reader):
        super(Controller, self).__init__(name)
        # self.logger.debug("%s [%s::__init__()]" % (self.name, self.__class__.__name__))
        self.w, self.r = writer, reader

        
    def log_message(self, function_name: str, feedback_message=""):
        self.logger.debug("%s [%s::%s][%s]" % (self.name, function_name, self.__class__.__name__, feedback_message))

    def setup(self):
        # self.log_message("setup()")
        self.Driver = ControllerModel(self.w, self.r)


    def initialise(self):
        # self.log_message("initialise()")
        pass

    def update(self):
        keyboard = self.r.device.keyboard
        key = keyboard.getKey()
        while(keyboard.getKey() != -1): pass

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