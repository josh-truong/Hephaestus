"""
localization.py

Created on Fri Nov 19 2022
@Lead: Joshua Truong
"""

import math
import numpy as np
from .utils import Pose

class Localization:

    def __init__(self, m):
        print("=== Localization Component Initialized...")
        self.m = m
        self.Pose = Pose(0, 0, 0)


    def get_pose(self):
        """
        Tier 1 - Localization
        Uses Webots gps/compass to obtain robot's pose
        """
        gps     = self.m.Device.gps
        compass = self.m.Device.compass

        n = compass.getValues()
        rad = ((math.atan2(n[0], n[1])))

        pose_x = -gps.getValues()[0]
        pose_y = -gps.getValues()[1]
        pose_theta = rad
        
        self.Pose = Pose(pose_x, pose_y, pose_theta)
        return self.Pose

    def get_position_error(self, goal: Pose):
        """
        Calculates the Euclidean distance rho between the current and goal pose.
        """
        rho = math.sqrt((self.Pose.x - goal.x)**2 + (self.Pose.y - goal.y)**2)
        return rho

    def get_bearing_error(self, goal: Pose):
        """
        Calculates the angle alpha between the robot orientation and the direction of the goal.
        """
        alpha = math.atan2(goal.y - self.Pose.y, goal.x - self.Pose.x)
        alpha += (math.pi if (alpha < 0) else -math.pi) - self.Pose.theta
        return alpha

    def get_heading_error(self, goal: Pose):
        """
        Calculates the angle eta between the orientation of the robot and the goal.
        """
        eta = goal.theta - self.Pose.theta
        return eta

    def update_odometry(self, vL, vR, print_pose=False):
        """
        Tier 2 - Localization
        Forward Kinematics - Odometry
        Computes angular and forward speed

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
        ts = self.m.Device.get_timestep()

        rConst = self.m.rConst
        MAX_SPEED    = rConst.MAX_SPEED
        MAX_SPEED_MS = rConst.MAX_SPEED_MS
        AXLE_LENGTH  = rConst.AXLE_LENGTH

        vL, vR = self.m.RobotController.get_wheel_velocity()
        self.Pose.x -= (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*ts/1000.0*math.cos(self.Pose.theta)
        self.Pose.y -= (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*ts/1000.0*math.sin(self.Pose.theta)
        self.Pose.theta += (vR-vL)/AXLE_LENGTH/MAX_SPEED*MAX_SPEED_MS*ts/1000.0

        if (print_pose):
            print(f"X: {self.Pose.x:.2f} Z: {self.Pose.y:.2f} Theta: {self.Pose.theta:2f}")