import math
import numpy as np

class Localization:

    def __init__(self, MAX_SPEED, MAX_SPEED_MS, AXLE_LENGTH):
        self.MAX_SPEED    = MAX_SPEED
        self.MAX_SPEED_MS = MAX_SPEED_MS
        self.AXLE_LENGTH  = AXLE_LENGTH

        self.pose_x     = 0
        self.pose_y     = 0
        self.pose_theta = 0

        self.vL = 0
        self.vR = 0

    def get_pose(self, gps, compass):
        """
        Uses Webots gps/compass to obtain robot's pose
        """
        self.pose_x = -gps.getValues()[0]
        self.pose_y = -gps.getValues()[1]

        n = compass.getValues()
        rad = ((math.atan2(n[0], n[1])))
        self.pose_theta = rad

        return self.pose_x, self.pose_y, self.pose_theta

    def update_odometry(self, vL, vR, ts, print_pose=False):
        """
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

        self.pose_x     += (x_r*math.cos(self.pose_theta))*ts
        self.pose_y     -= (x_r*math.sin(self.pose_theta))*ts
        self.pose_theta += omega_r*ts

        if (print_pose):
            print(f"X: {self.pose_x:.2f} Z: {self.pose_y:.2f} Theta: {self.pose_theta:2f}")