import py_trees
from .models import Vision
from .models import SpeedController
import numpy as np

class AlignToObject(py_trees.behaviour.Behaviour):
    def __init__(self, name, writer, reader):
        super(AlignToObject, self).__init__(name)
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
        centroids, blobs, img_mask = self.vision.detect(self.camera.getImageArray(), toggleShow=False)
        if (centroids is None or centroids == []): return py_trees.common.Status.SUCCESS
        idx = np.argmax([len(blob) for blob in blobs])
        x = centroids[idx][0]
        speed = self.r.constants.robot.MAX_SPEED
        margin = 10
        # if (x - ((240//2) + margin) < 0):
        #     vL, vR = speed*0.2, -speed*0.2
        # elif ():
        #     vL, vR = -speed*0.2, speed*0.2
        # self.w.robot.vL, self.w.robot.vR = vL, vR
        # self.controller.set_wheel_joint_vel(vL, vR)
        # self.controller.localization.update_odometry()
        pass

    def update(self):
        self.feedback_message = ""
        self.log_message("update()", self.feedback_message)
        return py_trees.common.Status.SUCCESS
        
    def terminate(self, new_status):
        # self.log_message("terminate()", "%s->%s" % (self.status, new_status))
        pass