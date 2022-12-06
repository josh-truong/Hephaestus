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
        self.logger.debug("%s [%s::__init__()]" % (self.name, self.__class__.__name__))
        self.w, self.r = writer, reader

        def convertMapToConfigSpace(map):
            kernel = np.ones((10,10))
            map = convolve2d(map, kernel, mode='same')
            map[map > 0] = 1
            return map
        
        robotPose = self.r.robot.pose
        def get_display_coords(x, y, display=(360, 360), world=(30, 15)):
            x = (display[0]*0.5) - (x * (display[0]/world[0]))
            y = display[1] - ((display[1]*0.5) - (y * (display[1]/world[1])))
            x, y = np.clip(x, 0, display[0]-1), np.clip(y, 0, display[1]-1)
            return [int(x), int(y)]

        x, y = get_display_coords(robotPose.x, -1 * robotPose.y)

        randPoint = np.random.randint(0,360,2)
        
        planner = Planning()
        map = convertMapToConfigSpace(self.r.env.map.map)
        nodes = planner.rrt([x, y], randPoint, 1000, 10, map)
        waypoints = planner.getWaypoints(nodes)
        self.w.env.waypoints = waypoints
        self.w.env.state = 0
        self.w.env.rerun_rrt = False

        
    def log_message(self, function_name: str, feedback_message=""):
        self.logger.debug("%s [%s::%s][%s]" % (self.name, function_name, self.__class__.__name__, feedback_message))

    def setup(self):
        self.log_message("setup()")
        self.Driver = ControllerModel(self.w, self.r)


    def initialise(self):
        # self.log_message("initialise()")
        pass

    def update(self):
        keyboard = self.r.device.keyboard
        key = keyboard.getKey()
        while(keyboard.getKey() != -1): pass

        vL, vR = 0, 0
        controller_type = self.r.controller_type
        velocity_rate = self.r.robot.velocity_rate
        if (controller_type == 'manual'):
            vL, vR = self.Driver.manual(key)
            vL, vR = vL*velocity_rate, vR*velocity_rate
        elif (controller_type == 'autonomous'):
            vL, vR = self.Driver.autonomous(velocity_rate)

        self.w.robot.vL, self.w.robot.vR = vL, vR
        self.Driver.set_wheel_joint_vel(vL, vR)
        self.Driver.localization.update_odometry()

        self.feedback_message = f"Left wheel velocity: {vL:.2f} Right wheel velocity: {vR:.2f}"
        self.log_message("update()", self.feedback_message)
        return py_trees.common.Status.SUCCESS
        
    def terminate(self, new_status):
        # self.log_message("terminate()", "%s->%s" % (self.status, new_status))
        pass