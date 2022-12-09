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
        if (key == ord('R')):
            self.w.env.rerun_rrt = True
        elif (key == ord('F')):
            def expand(img_mask, cur_coord):
                coordinates_in_blob = []
                coordinate_list = [cur_coord]
                while len(coordinate_list) > 0:
                    (y, x) = coordinate_list.pop()
                    if y < 0 or x < 0 or y >= img_mask.shape[0] or x >= img_mask.shape[1]:
                        continue
                    if img_mask[y, x] == 0.0:
                        continue
                    img_mask[y,x] = 0
                    coordinates_in_blob.append([x,y])
                    coordinate_list.extend([(y-1, x),(y+1, x),(y, x-1),(y, x+1)])
                return np.asarray(coordinates_in_blob)

            def get_blobs(img_mask):
                img_mask_height, img_mask_width = img_mask.shape[0], img_mask.shape[1]
                blobs_list, blobs_size = [], []
                img_mask_copy = img_mask.copy()
                for idx in range(img_mask_height*img_mask_width):
                    y, x = idx // img_mask_width, idx % img_mask_width
                    if (img_mask[y][x] > 0):
                        blob_coords = expand(img_mask_copy, [y, x])
                        if (len(blob_coords)):
                            blobs_list.append(blob_coords)
                            blobs_size.append(len(blob_coords))
                return np.array(blobs_list), np.array(blobs_size)
            
            blobs_list, blobs_size = get_blobs(self.r.env.map.map.copy())
            outlier_idx = np.where(blobs_size < 30)
            outliers = blobs_list[outlier_idx]
            if (outliers != []):
                for x, y in np.vstack(outliers):
                    self.w.env.map.update_pixel(x,y,0)
        elif (key == ord('Q')):
            self.w.env.behavior_state += 1

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