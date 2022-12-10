import py_trees
import numpy as np
from .models import SpeedController
from .models import Vision
from .models import DisplayOverlays
from .bt_move_forward import MoveForward
from .bt_align_to_object import AlignToObject

class LockAndLoad(py_trees.behaviour.Behaviour):
    def __init__(self, name, writer, reader):
        super(LockAndLoad, self).__init__(name)
        # self.logger.debug("%s [%s::__init__()]" % (self.name, self.__class__.__name__))
        self.w, self.r = writer, reader
        self.root = py_trees.composites.Sequence("Sequence")
        self.root.add_child(AlignToObject("Align to Object", self.w, self.r))
        # self.root.add_child(MoveForward("Moving to Object", self.w, self.r))
        self.root.setup_with_descendants()

    def log_message(self, function_name: str, feedback_message=""):
        self.logger.debug("%s [%s::%s][%s]" % (self.name, function_name, self.__class__.__name__, feedback_message))

    def setup(self):
        # self.log_message("setup()")
        self.driver = SpeedController(self.w, self.r)
        self.display = DisplayOverlays(self.w, self.r)
        self.camera = self.r.device.meta_camera
        width, height = self.camera.getWidth(), self.camera.getHeight()
        self.vision = Vision(width, height)
        self.controller = SpeedController(self.w, self.r)
        self.low_frequency = 50
        self.high_frequency = 30
        self.frequency = self.high_frequency
        self.counter = 0

    def initialise(self):
        self.log_message("initialise()")

    def update(self):
        self.log_message("update()", f"Next Vision Checking in {self.frequency-self.counter}. Current frequency: {self.frequency}")
        self.counter += 1
        if (self.counter%self.frequency == 0):
            self.counter = 0
            centroids, blobs, img_mask = self.vision.detect(self.camera.getImageArray(), toggleShow=False)
            if (centroids is None or centroids == []): return py_trees.common.Status.SUCCESS
            self.frequency = self.high_frequency
            self.counter = self.frequency
            idx = np.argmax([len(blob) for blob in blobs])
            centroid = centroids[idx]
            print(centroid, self.counter, self.frequency)
            robot = self.r.robot.robot
            while (robot.step(int(robot.getBasicTimeStep())) != -1):
                self.root.tick_once()
                
        else:
            self.frequency = self.low_frequency

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        # self.log_message("terminate()", "%s->%s" % (self.status, new_status))
        pass
