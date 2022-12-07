import py_trees
import numpy as np
from scipy.signal import convolve2d
from .models import ConfigSpace
from .models import DisplayOverlays
import matplotlib.pyplot as plt

class ObstacleAvoidance(py_trees.behaviour.Behaviour):
    def __init__(self, name, writer, reader):
        super(ObstacleAvoidance, self).__init__(name)
        self.logger.debug("%s [%s::__init__()]" % (self.name, self.__class__.__name__))
        self.w, self.r = writer, reader

    def log_message(self, function_name: str, feedback_message=""):
        self.logger.debug("%s [%s::%s][%s]" % (self.name, function_name, self.__class__.__name__, feedback_message))

    def setup(self):
        self.log_message("setup()")
        self.frequency = 200
        self.counter = 0
        self.display = DisplayOverlays(self.w, self.r)
        self.config_map = ConfigSpace().run(self.r.env.map.map)

    def initialise(self):
        self.log_message("initialise()")
        self.counter = 0

    def update(self):

        def get_lidar_readings():
            lidar = self.r.device.lidar
            lidar_sensor_readings = lidar.getRangeImage()
            lidar_sensor_readings = lidar_sensor_readings[83:len(lidar_sensor_readings)-83]
            return lidar_sensor_readings
        # Get the 25th percentile of lidar readings
        lidar_readings = np.array(get_lidar_readings())
        lidar_idx = np.where(lidar_readings <= np.percentile(lidar_readings, 25))[0]
        mean_depth = np.mean(lidar_readings[lidar_idx])
        self.log_message("update()", f"Nearest object: {mean_depth:.3f} m.")

        if (mean_depth < 1):
            ### ADD STEERING FUNCTION HERE
            left_readings = np.where(lidar_idx < len(lidar_readings)//2)[0]
            right_readings = np.where(lidar_idx > len(lidar_readings)//2)[0]
            
            if (len(left_readings) > len(right_readings)):
                self.log_message("update()", "Obstacle Detected on the left!")
                print("Obstacle Detected on the left!")
            else:
                self.log_message("update()", "Obstacle Detected on the right!")
                print("Obstacle Detected on the right!")

        self.log_message("update()", f"Next state check in {self.frequency-self.counter}.")
        self.counter += 1
        if (self.counter%self.frequency == 0):
            self.counter = 0
            self.config_map = ConfigSpace().run(self.r.env.map.map)
            plt.imshow(self.config_map)
            plt.show()

        waypoints = self.r.env.waypoints
        for i in range(self.r.env.state_step*2):
            i = np.clip(self.r.env.state-i, 0, len(waypoints)-1)
            x, y = waypoints[i]
            x, y = self.display.get_display_coords(x, y)
            if (self.config_map[y][x] == 1):
                print("Rerun rrt")
            print(i, self.config_map[y][x])
        
        return py_trees.common.Status.SUCCESS

        
        
    def terminate(self, new_status):
        self.log_message("terminate()", "%s->%s" % (self.status, new_status))