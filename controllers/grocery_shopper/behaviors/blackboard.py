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
            "device/left_gripper_enc", "device/right_gripper_enc", "device/gps", 
            "device/compass", "device/lidar", "device/display", "device/camera", 
            "device/keyboard", 
            "env/map", "env/refresh_hz", "env/ftol", "env/waypoints", "env/state", 
            "env/goal",
            "ik/rtol", "ik/atol", "ik/etol", "ik/p1", "ik/p2", "ik/p3",
            "robot/pose","robot/robot", "robot/parts", "robot/vL", "robot/vR", 
            "robot/velocity_rate", "robot/ts",
        }
        for key in keys:
            writer.register_key(key=key, access=py_trees.common.Access.WRITE)
            reader.register_key(key=key, access=py_trees.common.Access.READ)

        # Setting variables
        writer.debug = False
        writer.controller_type = 'manual'
        # writer.controller_type = 'autonomous'

        writer.constants.robot = RobotConstants()
        writer.constants.lidar = LidarConstants()

        writer.robot.pose = Pose()
        writer.robot.velocity_rate = 0.6
        writer.robot.vL = 0
        writer.robot.vR = 0
        writer.robot.ts = 0

        writer.env.map = Map()
        # writer.env.map.map = np.load('C:\\Users\\joshk\\OneDrive\\Desktop\\CSCI 3302 - Intro to Robotics\\Hephaestus\\controllers\\grocery_shopper\\assets\\map.npy')
        writer.env.refresh_hz = 50
        writer.env.ftol = 1
        writer.env.state = 0
        writer.env.goal = None
        writer.env.waypoints = None
        writer.env.waypoints = [
            (  4.80,  0.00,  0.00),
            (  4.79, -2.22,  0.00),
            (-12.83, -2.30,  0.00),
            (-12.78,  2.03,  0.00),
            (  4.89,  1.80,  0.00),
            (  4.94,  5.68,  0.00),
            (-13.01,  5.51,  0.00),
            (-13.24, -5.64,  0.00)
        ]

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
        writer.device.camera = device.camera
        writer.device.keyboard = device.keyboard

        writer.robot.vL = 0
        writer.robot.vR = 0