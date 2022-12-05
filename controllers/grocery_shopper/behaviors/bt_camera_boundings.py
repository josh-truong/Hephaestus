import py_trees
import numpy as np
from .models import EdgeDetection, Vision


class CameraBounds(py_trees.behaviour.Behaviour):
    def __init__(self, name, writer, reader):
        super(CameraBounds, self).__init__(name)
        self.logger.debug("%s [%s::__init__()]" % (self.name, self.__class__.__name__))
        self.w, self.r = writer, reader

    def log_message(self, function_name: str, feedback_message=""):
        self.logger.debug("%s [%s::%s][%s]" % (self.name, function_name, self.__class__.__name__, feedback_message))

    def setup(self):
        self.log_message("setup()")
        self.camera_frequency = self.r.env.refresh_hz*2
        self.camera_counter = 0
        self.detection = EdgeDetection(self.w, self.r)
        self.vision = Vision(self.w, self.r)

    def initialise(self):
        # self.log_message("initialise()")
        pass

    def update(self):
        self.camera_counter += 1
        self.feedback_message = f"Camera detection[{self.camera_frequency - self.camera_counter}]."
        if (self.camera_counter%self.camera_frequency == 0):
            self.camera_counter = 0
            centroids, blobs = self.vision.detect(toggleShow=True)
            if (len(centroids) != 0):
                for centroid in centroids:
                    y, x = centroid
                    range_finder = self.r.device.range_finder
                    range_width = range_finder.getWidth() 
                    image_bytes = range_finder.getRangeImage(data_type="buffer")
                    range_image = np.frombuffer(image_bytes, dtype=np.float32)
                    depth = range_image[int(y*range_width + x)]

                
            # map_bounds = self.detection.get_obstacle_bound(img_mask, 1)
            # self.detection.draw_bounds(map_bounds, 0xFFFF00)
            
        
        self.log_message("update()", self.feedback_message)
        return py_trees.common.Status.SUCCESS
        
    def terminate(self, new_status):
        # self.log_message("terminate()", "%s->%s" % (self.status, new_status))
        pass
