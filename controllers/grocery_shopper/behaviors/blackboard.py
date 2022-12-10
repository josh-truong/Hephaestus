"""
blackboard.py

Last updated on Fri Dec 9 2022
@Lead: Joshua Truong
"""


import py_trees
import numpy as np
from controller import Supervisor

from .models import RobotConstants, LidarConstants
from .models import Map, Pose
from .models import Device

class Blackboard:
    def __init__(self):
        print("=== Initializing Blackboards...")
        # Blackboards for writing
        writer = py_trees.blackboard.Client(name="Writer")
        # Blackboard for reading
        reader = py_trees.blackboard.Client(name="Reader")

        keys = {
            "debug", "controller_type",
            "constants/robot", "constants/lidar",
            "device/left_gripper_enc", "device/right_gripper_enc", 
            "device/keyboard", 
            "device/gps", "device/compass", 
            "device/lidar", "device/disable_lidar", 
            "device/meta_camera", "device/range_finder", 
            "device/display", "device/depth_display", 
            "env/map", "env/refresh_hz", "env/ftol", "env/waypoints", "env/state", 
            "env/goal", "env/rerun_rrt", "env/state_step",
            "env/num_completed_paths", "env/behavior_state", "env/vaiable_waypoints",
            "env/max_completed_paths", "env/xmax_boundary", "env/ymax_boundary",
            "env/num_objects",
            "env/object_location", "env/kmeans_location", "env/kmeans_state",
            "ik/rtol", "ik/atol", "ik/etol", "ik/p1", "ik/p2", "ik/p3",
            "robot/pose","robot/robot", "robot/parts", "robot/vL", "robot/vR", 
            "robot/velocity_rate", "robot/ts", "robot/reverse", "robot/msg"
        }
        for key in keys:
            writer.register_key(key=key, access=py_trees.common.Access.WRITE)
            reader.register_key(key=key, access=py_trees.common.Access.READ)

        # Setting variables
        # py_trees.logging.level = py_trees.logging.Level.DEBUG
        writer.debug = False
        writer.env.behavior_state = 1
        # writer.controller_type = 'manual'
        writer.controller_type = 'autonomous'

        writer.constants.robot = RobotConstants()
        writer.constants.lidar = LidarConstants()

        writer.robot.pose = Pose()
        writer.robot.velocity_rate = 0.6
        writer.robot.vL = 0
        writer.robot.vR = 0
        writer.robot.ts = 0
        writer.robot.reverse = False
        writer.robot.msg = ""

        writer.env.map = Map()
        writer.env.map.map = np.load('assets/map.npy')
        writer.env.xmax_boundary = 360
        writer.env.ymax_boundary = 360
        writer.env.num_objects = 12
        writer.env.rerun_rrt = True
        writer.env.num_completed_paths = 0
        writer.env.max_completed_paths = 10
        writer.env.refresh_hz = 50
        writer.env.ftol = 1
        writer.env.state = 0
        writer.env.state_step = 15
        writer.env.goal = None
        writer.env.waypoints = None
        writer.env.vaiable_waypoints = []
        # writer.env.vaiable_waypoints = np.load('C:\\Users\\joshk\\OneDrive\\Desktop\\CSCI 3302 - Intro to Robotics\\Hephaestus\\controllers\\grocery_shopper\\assets\\viable_waypoints.npy').tolist()
        writer.env.kmeans_state = 0
        writer.env.kmeans_location = []
        writer.env.object_location = []
        writer.env.object_location = np.load('assets/object_location.npy').tolist()

        writer.ik.rtol = 0.2
        writer.ik.atol = 3.14/10
        writer.ik.etol = 3.14/10

        writer.ik.p1 = 0.4   # dist
        writer.ik.p2 = 0.7   # bearing
        writer.ik.p3 = 0.05  # heading

        self.set_devices(writer)
        self.writer, self.reader = writer, reader
        
        # print(py_trees.display.unicode_blackboard_activity_stream())

    def get(self):
        return self.writer, self.reader

    def set_devices(self, writer):
        device = Device()
        writer.robot.robot = device.robot
        writer.robot.parts = device.robot_parts
        
        writer.device.left_gripper_enc = device.left_gripper_enc
        writer.device.right_gripper_enc = device.right_gripper_enc
        writer.device.gps = device.gps
        writer.device.compass = device.compass
        writer.device.lidar = device.lidar
        writer.device.display = device.display
        writer.device.depth_display = device.depth_display
        writer.device.meta_camera = device.meta_camera
        writer.device.range_finder = device.range_finder
        writer.device.keyboard = device.keyboard
        writer.device.disable_lidar = False

        writer.robot.vL = 0
        writer.robot.vR = 0