import py_trees
import numpy as np
import math
from .models import ObjectBound, Vision
import matplotlib.pyplot as plt
from .models import DisplayOverlays

class CameraVision(py_trees.behaviour.Behaviour):
    def __init__(self, name, writer, reader):
        super(CameraVision, self).__init__(name)
        # self.logger.debug("%s [%s::__init__()]" % (self.name, self.__class__.__name__))
        self.w, self.r = writer, reader

    def log_message(self, function_name: str, feedback_message=""):
        self.logger.debug("%s [%s::%s][%s]" % (self.name, function_name, self.__class__.__name__, feedback_message))

    def setup(self):
        # self.log_message("setup()")
        self.camera_frequency = self.r.env.refresh_hz
        self.camera_counter = 0
        self.objectBound = ObjectBound()
        self.display = DisplayOverlays(self.w, self.r)
        self.camera = self.r.device.meta_camera
        width, height = self.camera.getWidth(), self.camera.getHeight()
        self.vision = Vision(width, height)

    def initialise(self):
        # self.log_message("initialise()")
        pass

    def update(self):
        def get_lidar_readings():
            lidar = self.r.device.lidar
            lidar_sensor_readings = lidar.getRangeImage()
            lidar_sensor_readings = lidar_sensor_readings[83:len(lidar_sensor_readings)-83]
            return np.array(lidar_sensor_readings)

        self.camera_counter += 1
        self.feedback_message = f"Object Detection in {self.camera_frequency - self.camera_counter}."
        if (self.camera_counter%self.camera_frequency == 0):
            self.camera_counter = 0
            centroids, blobs, img_mask = self.vision.detect(self.camera.getImageArray(), toggleShow=False)

            range_finder = self.r.device.range_finder
            range_width = range_finder.getWidth() 
            image_bytes = range_finder.getRangeImage(data_type="buffer")
            range_image = np.frombuffer(image_bytes, dtype=np.float32)

            # Update depth display
            self.display.update_depth_display(range_image, range_width)
            object_blobs = self.objectBound.return_blobs(img_mask)
            object_bounds = self.objectBound.return_bounds(blobs)
            self.display.draw_object_bounds(self.r.device.depth_display, object_bounds)
            self.display.draw_estimated_location_on_map()

            # Estimate object position
            for i, centroid in enumerate(centroids):
                y, x = centroid
                depth = range_image[int(y*range_width + x)]

                if (depth < range_finder.getMinRange() or range_finder.getMaxRange() < depth): continue

                lidar_readings = get_lidar_readings()
                n = len(lidar_readings)

                is_left = (centroid[0] < 120)
                readings = lidar_readings[:n//2] if(is_left) else lidar_readings[n//2:]
                idx = np.argmin(np.abs(readings-depth))
                idx = idx if (is_left) else idx+(n//2)
                offset = self.r.constants.lidar.OFFSETS
                alpha = offset[idx]
                pose = self.r.robot.pose
                rx = -math.cos(alpha)*depth + 0.202
                ry = math.sin(alpha)*depth -0.004
                # Convert detection from robot coordinates into world coordinates
                wx =  math.cos(pose.theta)*rx - math.sin(pose.theta)*ry + pose.x
                wy =  +(math.sin(pose.theta)*rx + math.cos(pose.theta)*ry) + pose.y
                self.w.env.object_location.append([wx, wy, depth])
            
        self.log_message("update()", self.feedback_message)
        return py_trees.common.Status.SUCCESS
        
    def terminate(self, new_status):
        # self.log_message("terminate()", "%s->%s" % (self.status, new_status))
        pass
