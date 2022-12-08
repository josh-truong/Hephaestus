import py_trees
import numpy as np
from scipy.signal import convolve2d
from .models import ConfigSpace
from .models import DisplayOverlays
import matplotlib.pyplot as plt
from .models import ControllerModel
from .models import Localization

from .bt_reverse import Reverse
from .bt_rrt import RRT

class ObstacleAvoidance(py_trees.behaviour.Behaviour):
    def __init__(self, name, writer, reader):
        super(ObstacleAvoidance, self).__init__(name)
        # self.logger.debug("%s [%s::__init__()]" % (self.name, self.__class__.__name__))
        self.w, self.r = writer, reader
        self.reverse_behavior = py_trees.composites.Sequence("Reverse Sequence")
        self.reverse_behavior.add_child(Reverse("Reversing", self.w, self.r))
        self.reverse_behavior.add_child(RRT("RRT for reverse", self.w, self.r))
        self.reverse_behavior.setup_with_descendants()

    def log_message(self, function_name: str, feedback_message=""):
        self.logger.debug("%s [%s::%s][%s]" % (self.name, function_name, self.__class__.__name__, feedback_message))

    def setup(self):
        self.log_message("setup()")
        self.frequency = 200
        self.counter = 0
        self.display = DisplayOverlays(self.w, self.r)
        self.config_map = ConfigSpace().run(self.r.env.map.map)
        self.Driver = ControllerModel(self.w, self.r)
        self.localization = Localization(self.w, self.r)
        self.linspace_waypoints = None
        self.prev_endstate = None
        self.prev_pose = self.r.robot.pose
        self.n_stuck = 0
        self.max_stuck = 500

        

    def initialise(self):
        # self.log_message("initialise()")
        pass

    def update(self):

        # pose =  self.r.robot.pose
        # if (self.n_stuck%self.max_stuck==0 and self.n_stuck!=0):
        #     self.w.env.rerun_rrt = True
        #     self.n_stuck = 0
        #     print("Max stuck")
        # elif (abs(pose.x-self.prev_pose.x)+abs(pose.y-self.prev_pose.y) < 1):
        #     self.n_stuck += 1
        #     print(f"Detected Stuck {self.n_stuck} {abs(pose.x-self.prev_pose.x)+abs(pose.y-self.prev_pose.y)}")
        # elif (self.counter%50 == 0):
        #     self.prev_pose =  self.r.robot.pose
        #     print("update prev pose")
        # else:
        #     self.n_stuck = 0
        #     print(f"not stuck {abs(pose.x-self.prev_pose.x)+abs(pose.y-self.prev_pose.y)}")

        # self.log_message("update()", f"Next state check in {self.frequency-self.counter}.")
        # self.counter += 1
        # # Re-convolve map based on frequency since it is a performance problem
        # if (self.counter%self.frequency == 0):
        #     self.counter = 0
        #     self.config_map = ConfigSpace().run(self.r.env.map.map)

        # # # Recheck if path is still valid
        # waypoints = self.r.env.waypoints
        # n = len(waypoints)
        # state, state_step = self.r.env.state, self.r.env.state_step
        # endstate = np.clip(state+state_step*3, 0, n-1)
        # if (self.prev_endstate != endstate):
        #     self.prev_endstate = endstate

        #     startstate = np.clip(state-state_step, 0, n-1)
        #     startpoint, endpoint = waypoints[startstate], waypoints[endstate]
        #     manhatten_distance = int(abs(startpoint[0]-endpoint[0]) + abs(startpoint[1]-endpoint[1]))
        #     self.linspace_waypoints = np.linspace(startpoint, endpoint, manhatten_distance)

        # for x, y in self.linspace_waypoints:
        #     x, y = self.display.get_display_coords(x, y)
        #     if (self.config_map[y][x] == 1):
        #         self.log_message("update()", "Invalid path found! Rerunning RRT.")
        #         self.w.env.rerun_rrt = True
        #         self.w.env.state = np.clip(self.r.env.state-2, 0, n-1)
        #         break

        self.reverse_behavior.tick_once()






        # Steer robot if obstacle is detected
        # if (mean_depth < 0.9):
            # left_readings = np.where(lidar_idx < len(lidar_readings)//2)[0]
            # right_readings = np.where(lidar_idx > len(lidar_readings)//2)[0]
            # velocity_rate = self.r.robot.velocity_rate
            # speed = self.r.constants.robot.MAX_SPEED
            # if (len(left_readings) > len(right_readings)):
            #     self.w.robot.vL, self.w.robot.vR = 0.2*speed, -0.2*speed
            # else:
            #     self.w.robot.vL, self.w.robot.vR = -0.2*speed, 0.2*speed

            # self.Driver.set_wheel_joint_vel(self.r.robot.vL, self.r.robot.vR)
            # self.Driver.localization.update_odometry()

            # # Find the farthest waypoint given a radius
            # waypoints = np.array(self.r.env.waypoints)
            # pose =  self.r.robot.pose
            # pose = np.array([pose.x, pose.y])
            # state = np.argmin(np.linalg.norm(waypoints-pose, axis=1))
            # self.w.env.state = np.clip(state+1, 0, len(waypoints)-1)
            # return py_trees.common.Status.RUNNING
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        # self.log_message("terminate()", "%s->%s" % (self.status, new_status))
        pass