import py_trees
from .models import SpeedController
import numpy as np

class MoveBackward(py_trees.behaviour.Behaviour):
    def __init__(self, name, writer, reader):
        super(MoveBackward, self).__init__(name)
        self.logger.debug("%s [%s::__init__()]" % (self.name, self.__class__.__name__))
        self.w, self.r = writer, reader

    def log_message(self, function_name: str, feedback_message=""):
        self.logger.debug("%s [%s::%s][%s]" % (self.name, function_name, self.__class__.__name__, feedback_message))

    def setup(self):
        # self.log_message("setup()")
        self.controller = SpeedController(self.w, self.r)

    def initialise(self):
        # self.log_message("initialise()")
        pass

    def update(self):
        self.log_message("update()", "Moving Backward")

        def get_lidar_readings():
            lidar = self.r.device.lidar
            lidar_sensor_readings = lidar.getRangeImage()
            lidar_sensor_readings = lidar_sensor_readings[83:len(lidar_sensor_readings)-83]
            return lidar_sensor_readings

        robot = self.r.robot.robot
        while (robot.step(int(robot.getBasicTimeStep())) != -1):
            # Get the 25th percentile of lidar readings
            lidar_readings = np.array(get_lidar_readings())
            lidar_idx = np.where(lidar_readings <= np.percentile(lidar_readings, 25))[0]
            mean_depth = np.mean(lidar_readings[lidar_idx])
            print(mean_depth)
            if (mean_depth < 1.5):
                speed = self.r.constants.robot.MAX_SPEED
                vL, vR = -1.05, -1.05
                self.w.robot.vL, self.w.robot.vR = vL, vR
                self.controller.set_wheel_joint_vel(vL, vR)
                self.controller.localization.update_odometry()
            else:
                break
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        # self.log_message("terminate()", "%s->%s" % (self.status, new_status))
        pass
