import py_trees
from .models import Vision
from .models import SpeedController
import numpy as np
import math

class RotateRight90(py_trees.behaviour.Behaviour):
    """
    Rotate to 90 degrees to right
    """
    def __init__(self, name, writer, reader):
        super(RotateRight90, self).__init__(name)
        # self.logger.debug("%s [%s::__init__()]" % (self.name, self.__class__.__name__))
        self.w, self.r = writer, reader

    def log_message(self, function_name: str, feedback_message=""):
        self.logger.debug("%s [%s::%s][%s]" % (self.name, function_name, self.__class__.__name__, feedback_message))

    def setup(self):
        # self.log_message("setup()")
        self.camera = self.r.device.meta_camera
        width, height = self.camera.getWidth(), self.camera.getHeight()
        self.vision = Vision(width, height)
        self.controller = SpeedController(self.w, self.r)

    def initialise(self):
        robot = self.r.robot.robot
        while (robot.step(int(robot.getBasicTimeStep())) != -1):
            compass = self.r.device.compass.getValues()
            degrees = np.degrees(math.atan2(compass[0], compass[1]))
            if (-88 <= degrees and degrees <= -92):
                break
            speed = self.r.constants.robot.MAX_SPEED
            vL, vR = 1.05, -1.05
            self.w.robot.vL, self.w.robot.vR = vL, vR
            self.controller.set_wheel_joint_vel(vL, vR)
            self.controller.localization.update_odometry()
        return py_trees.common.Status.SUCCESS

    def update(self):
        self.feedback_message = ""
        self.log_message("update()", self.feedback_message)
        return py_trees.common.Status.SUCCESS
        
    def terminate(self, new_status):
        # self.log_message("terminate()", "%s->%s" % (self.status, new_status))
        pass