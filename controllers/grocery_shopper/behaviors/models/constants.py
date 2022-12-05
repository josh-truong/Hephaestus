"""
constants.py

Last updated on Sun Dec 4 2022
@Lead: Joshua Truong
"""

import math
import numpy as np

class RobotConstants:
    def __init__(self):
        self.MAX_SPEED    = 7.0    # [rad/s]
        self.MAX_SPEED_MS = 0.633  # [m/s]
        self.AXLE_LENGTH  = 0.4044 # m

        self.MOTOR_LEFT  = 10
        self.MOTOR_RIGHT = 11
        self.N_PARTS     = 12
        # The Tiago robot has multiple motors, each identified by their names below
        self.PART_NAMES = (
            "head_2_joint", "head_1_joint", "torso_lift_joint", "arm_1_joint",
            "arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint",
            "arm_6_joint", "arm_7_joint", "wheel_left_joint", "wheel_right_joint",
            "gripper_left_finger_joint","gripper_right_finger_joint"
        )

    def __str__(self):
        return str(self.__dict__)

class LidarConstants:
    def __init__(self):
        self.ANGLE_BINS = 667
        self.SENSOR_MAX_RANGE = 5.5 # Meters
        self.ANGLE_RANGE = math.radians(240)

        self.OFFSETS = np.linspace(-self.ANGLE_RANGE/2., +self.ANGLE_RANGE/2., self.ANGLE_BINS)
        self.OFFSETS = self.OFFSETS[83:len(self.OFFSETS)-83] # Only keep lidar readings not blocked by robot chassis

    def __str__(self):
        return str(self.__dict__)