import py_trees
import numpy as np
from .models import SpeedController
from .models import Vision
from .models import DisplayOverlays

import math
from .bt_RotateLeft90 import RotateLeft90
from .bt_RotateRight90 import RotateRight90
from .bt_move_forward import MoveForward
from .bt_move_backward import MoveBackward


class LockAndLoad(py_trees.behaviour.Behaviour):
    def __init__(self, name, writer, reader):
        super(LockAndLoad, self).__init__(name)
        # self.logger.debug("%s [%s::__init__()]" % (self.name, self.__class__.__name__))
        self.w, self.r = writer, reader
        self.root_l = py_trees.composites.Sequence("Sequence")
        self.root_l.add_child(RotateLeft90("Rotate Left", self.w, self.r))
        self.root_l.add_child(MoveForward("Move Forward", self.w, self.r))
        self.root_l.add_child(RotateLeft90("Rotate To 90", self.w, self.r))
        self.root_l.add_child(MoveBackward("Backing up", self.w, self.r))
        self.root_l.setup_with_descendants()

        self.root_r = py_trees.composites.Sequence("Sequence")
        self.root_r.add_child(RotateRight90("Rotate Right", self.w, self.r))
        self.root_l.add_child(MoveForward("Move Forward", self.w, self.r))
        self.root_r.setup_with_descendants()

    def log_message(self, function_name: str, feedback_message=""):
        self.logger.debug("%s [%s::%s][%s]" % (self.name, function_name, self.__class__.__name__, feedback_message))

    def setup(self):
        # self.log_message("setup()")
        self.driver = SpeedController(self.w, self.r)
        self.display = DisplayOverlays(self.w, self.r)
        # self.camera = self.r.device.meta_camera
        # width, height = self.camera.getWidth(), self.camera.getHeight()
        self.left_cam = self.r.device.left_camera
        self.right_cam = self.r.device.right_camera
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
            margin = 10
            l_width, l_height = self.left_cam.getWidth(), self.left_cam.getHeight()
            r_width, r_height = self.right_cam.getWidth(), self.right_cam.getHeight()
            left_vision = Vision(l_width, l_height)
            right_vision = Vision(r_width, r_height)
            l_centroids, l_blobs, _ = left_vision.detect(self.left_cam.getImageArray(), toggleShow=False)
            r_centroids, r_blobs, _ = right_vision.detect(self.right_cam.getImageArray(), toggleShow=False)

            if (l_centroids != []):
                print(l_centroids)
                idx = np.argmax([len(blob) for blob in l_blobs])
                l_x = l_centroids[idx][0]//2
                screen_x = l_width // 2
                if (screen_x-margin < l_x and l_x < screen_x+margin):
                    print("Turning Left")
                    self.root_l.tick_once()
            elif (r_centroids != []):
                print(r_centroids)
                idx = np.argmax([len(blob) for blob in r_blobs])
                r_x = r_centroids[idx][0]//2
                screen_x = r_width // 2
                if (screen_x-margin < r_x and r_x < screen_x+margin):
                    print("Turning Right")
                    self.root_r.tick_once()

        compass = self.r.device.compass.getValues()
        degrees = np.degrees(math.atan2(compass[0], compass[1]))
        print(degrees)
        



            # if (r_centroids is not None or r_centroids != []):

            # self.frequency = self.high_frequency
            # self.counter = self.frequency
            
            
        # else:
        #     self.frequency = self.low_frequency

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        # self.log_message("terminate()", "%s->%s" % (self.status, new_status))
        pass
