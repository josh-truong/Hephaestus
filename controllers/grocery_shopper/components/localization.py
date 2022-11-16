import math
import numpy as np
from .utils import Pose

class Localization:

    def __init__(self, RobotConst):
        self.MAX_SPEED    = RobotConst.MAX_SPEED
        self.MAX_SPEED_MS = RobotConst.MAX_SPEED_MS
        self.AXLE_LENGTH  = RobotConst.AXLE_LENGTH

        self.pose = Pose(0, 0, 0)

        self.vL = 0
        self.vR = 0

    def get_pose(self, gps, compass):
        """
        Tier 1 - Localization
        Uses Webots gps/compass to obtain robot's pose
        """
        n = compass.getValues()
        rad = ((math.atan2(n[0], n[1])))

        pose_x = -gps.getValues()[0]
        pose_y = -gps.getValues()[1]
        pose_theta = rad
        
        self.pose = Pose(pose_x, pose_y, pose_theta)
        return self.pose

    def update_odometry(self, vL, vR, ts, print_pose=False):
        """
        Tier 2 - Localization
        Forward Kinematics - Odometry
        Computes angular and forward speed

        # Compact version but wastes cpu cycles
        pose_x += (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*ts/1000.0*math.cos(pose_theta)
        pose_y -= (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*ts/1000.0*math.sin(pose_theta)
        pose_theta += (vR-vL)/AXLE_LENGTH/MAX_SPEED*MAX_SPEED_MS*ts/1000.0

        # Matrix operation version
        velocity = np.array([
            [(vL+vR)/2/MAX_SPEED*MAX_SPEED_MS],
            [(vR-vL)/AXLE_LENGTH/MAX_SPEED*MAX_SPEED_MS]
        ])
        rot_mat = np.array([
            [np.cos(pose_theta), 0],
            [np.sin(pose_theta), 0],
            [0, 1]
        ])
        delta_velocity = (rot_mat @ velocity)
        pose_x, pose_y, pose_theta = delta_velocity*(ts/1000.0)
        """
        ts = ts/1000.0
        MAX_SPEED_RAD_M_S = self.MAX_SPEED*self.MAX_SPEED_MS
        x_r     = (vL+vR)/2/MAX_SPEED_RAD_M_S
        omega_r = (vR-vL)/self.AXLE_LENGTH/MAX_SPEED_RAD_M_S

        self.pose.x     += (x_r*math.cos(self.pose.theta))*ts
        self.pose.y     -= (x_r*math.sin(self.pose.theta))*ts
        self.pose.theta += omega_r*ts

        if (print_pose):
            print(f"X: {self.pose.x:.2f} Z: {self.pose.y:.2f} Theta: {self.pose.theta:2f}")