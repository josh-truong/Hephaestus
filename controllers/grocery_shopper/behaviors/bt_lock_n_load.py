import py_trees
from .models import SpeedController
from .models import Vision
from .models import DisplayOverlays


class LockAndLoad(py_trees.behaviour.Behaviour):
    def __init__(self, name, writer, reader):
        super(LockAndLoad, self).__init__(name)
        self.logger.debug("%s [%s::__init__()]" % (self.name, self.__class__.__name__))
        self.w, self.r = writer, reader
        # self.root = py_trees.composites.Sequence("Sequence")
        # self.root.setup_with_descendants()

    def log_message(self, function_name: str, feedback_message=""):
        self.logger.debug("%s [%s::%s][%s]" % (self.name, function_name, self.__class__.__name__, feedback_message))

    def setup(self):
        self.log_message("setup()")
        self.display = DisplayOverlays(self.w, self.r)
        self.camera = self.r.device.meta_camera
        width, height = self.camera.getWidth(), self.camera.getHeight()
        self.vision = Vision(width, height)
        self.controller = SpeedController(self.w, self.r)
        self.low_frequency = 100
        self.high_frequency = 30
        self.frequency = self.high_frequency
        self.counter = 0

    def initialise(self):
        self.log_message("initialise()")

    def update(self):
        self.log_message("update()", f"Next Vision Checking in {self.frequency-self.counter}. Current frequency: {self.frequency}")
        if (self.counter%self.frequency == 0):
            self.frequency = self.high_frequency
            centroids, blobs, img_mask = self.vision.detect(self.camera.getImageArray(), toggleShow=False)

            # Update depth display
            # self.display.update_depth_display(self.r.device.meta_camera, range_width)
            # object_blobs = self.objectBound.return_blobs(img_mask)
            # object_bounds = self.objectBound.return_bounds(blobs)
            # self.display.draw_object_bounds(self.r.device.depth_display, object_bounds)
            # self.display.draw_estimated_location_on_map()
        else:
            self.frequency = self.low_frequency



        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        # self.log_message("terminate()", "%s->%s" % (self.status, new_status))
        pass
