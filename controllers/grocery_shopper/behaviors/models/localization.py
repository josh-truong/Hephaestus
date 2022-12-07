"""
localization.py

Created on Fri Nov 19 2022
@Lead: Joshua Truong
"""

import math
import numpy as np
from .utils import Pose

class Localization:
    def __init__(self, writer, reader):
        self.w, self.r = writer, reader

    def get_pose(self):
        """
        Tier 1 - Localization
        Uses Webots gps/compass to obtain robot's pose
        """
        gps     = self.r.device.gps
        compass = self.r.device.compass

        n = compass.getValues()
        rad = math.atan2(n[0], n[1])

        pose_x = -gps.getValues()[0]
        pose_y = -gps.getValues()[1]
        pose_theta = rad
        
        return Pose(pose_x, pose_y, pose_theta)

    def get_position_error(self):
        """
        Calculates the Euclidean distance rho between the current and goal pose.
        """
        pose, goal = self.r.robot.pose, self.r.env.goal
        rho = math.sqrt((pose.x - goal.x)**2 + (pose.y - goal.y)**2)
        return rho

    def get_bearing_error(self):
        """
        Calculates the angle alpha between the robot orientation and the direction of the goal.
        """
        pose, goal = self.r.robot.pose, self.r.env.goal
        alpha = math.atan2(goal.y - pose.y, goal.x - pose.x) - pose.theta
        alpha += (math.pi if (alpha < 0) else -math.pi)
        return alpha

    def get_heading_error(self):
        """
        Calculates the angle eta between the orientation of the robot and the goal.
        """
        pose, goal = self.r.robot.pose, self.r.env.goal
        eta = goal.theta - pose.theta
        return eta

    def update_odometry(self):
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
        ts = int(self.r.robot.robot.getBasicTimeStep())
        self.w.robot.ts = ts

        MAX_SPEED    = self.r.constants.robot.MAX_SPEED
        MAX_SPEED_MS = self.r.constants.robot.MAX_SPEED_MS
        AXLE_LENGTH  = self.r.constants.robot.AXLE_LENGTH

        vL, vR = self.r.robot.vL, self.r.robot.vR
        pose = self.get_pose()
        pose.x -= (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*ts/1000.0*math.cos(pose.theta)
        pose.y -= (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*ts/1000.0*math.sin(pose.theta)
        pose.theta += (vR-vL)/AXLE_LENGTH/MAX_SPEED*MAX_SPEED_MS*ts/1000.0
        self.w.robot.pose = pose