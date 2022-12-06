import py_trees
import numpy as np
import math
from .models import EdgeDetection, Vision
import matplotlib.pyplot as plt


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
        def get_lidar_readings():
            lidar = self.r.device.lidar
            lidar_sensor_readings = lidar.getRangeImage()
            lidar_sensor_readings = lidar_sensor_readings[83:len(lidar_sensor_readings)-83]
            return np.array(lidar_sensor_readings)

        self.camera_counter += 1
        self.feedback_message = f"Camera detection[{self.camera_frequency - self.camera_counter}]."
        if (self.camera_counter%self.camera_frequency == 0):
            self.camera_counter = 0
            centroids, blobs = self.vision.detect(toggleShow=False)

            range_finder = self.r.device.range_finder
            range_width = range_finder.getWidth() 
            image_bytes = range_finder.getRangeImage(data_type="buffer")
            range_image = np.frombuffer(image_bytes, dtype=np.float32)


            # map = range_image.reshape(-1, range_width)
            # im = plt.imshow(map)
            # map = im.cmap(im.norm(im.get_array()))
            # map = np.delete(map, 3, 2)*255
            # depth_display = self.r.device.depth_display
            # ir = depth_display.imageNew(map.tolist(), depth_display.RGB)
            # depth_display.imagePaste(ir, 0, 0, False)
            # depth_display.imageDelete(ir)

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
                alpha = self.r.constants.lidar.OFFSETS[idx]
                pose = self.r.robot.pose
                rx = -math.cos(alpha)*depth + 0.202
                ry = math.sin(alpha)*depth -0.004
                # Convert detection from robot coordinates into world coordinates
                wx =  math.cos(pose.theta)*rx - math.sin(pose.theta)*ry + pose.x
                wy =  +(math.sin(pose.theta)*rx + math.cos(pose.theta)*ry) + pose.y
                self.w.env.object_location.append([wx, wy, depth])
                
                # print(len(blobs[i]), depth, wx, wy)




            # map_bounds = self.detection.get_obstacle_bound(img_mask, 1)
            # self.detection.draw_bounds(map_bounds, 0xFFFF00)
            
        
        self.log_message("update()", self.feedback_message)
        return py_trees.common.Status.SUCCESS
        
    def terminate(self, new_status):
        # self.log_message("terminate()", "%s->%s" % (self.status, new_status))
        pass
