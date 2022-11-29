"""
managerController.py

Created on Fri Nov 19 2022
@Lead: Joshua Truong
"""

from .constants import RobotConst, LidarConst

from .devices import Device
from .localization import Localization
from .mapping import Mapping
from .slam import SLAM
from .robotController import RobotController

class Manager:
    def __init__(self):
        """
        Accessible Attributes of Classes
        Class Localization
            - Pose: type Pose
        Class Pose
            - x, y, theta
        Mapping
            - Map: type Map (size 360x360)
            - robot_poses: list of previous poses [(x1,y1), ..., (xn,yn)]
        Slam
        Device
            - 
        RobotController
            - vL, vR
        """
        self.rConst = RobotConst()
        self.lConst = LidarConst()
        
        self.Localization    = None
        self.Mapping         = None
        self.Slam            = None
        self.Device          = None
        self.RobotController = None
