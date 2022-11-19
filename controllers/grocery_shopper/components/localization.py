import math
import numpy as np
from .utils import Pose
from .constants import RobotConst


class Localization:

    def __init__(self):
        print("=== Initializing Localization Component...")
        rConst = RobotConst()
        self.MAX_SPEED    = rConst.MAX_SPEED
        self.MAX_SPEED_MS = rConst.MAX_SPEED_MS
        self.AXLE_LENGTH  = rConst.AXLE_LENGTH

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
        self.pose.x -= (vL+vR)/2/self.MAX_SPEED*self.MAX_SPEED_MS*ts/1000.0*math.cos(self.pose.theta)
        self.pose.y -= (vL+vR)/2/self.MAX_SPEED*self.MAX_SPEED_MS*ts/1000.0*math.sin(self.pose.theta)
        self.pose.theta += (vR-vL)/self.AXLE_LENGTH/self.MAX_SPEED*self.MAX_SPEED_MS*ts/1000.0

        if (print_pose):
            print(f"X: {self.pose.x:.2f} Z: {self.pose.y:.2f} Theta: {self.pose.theta:2f}")