"""grocery controller."""
import matplotlib.pyplot as plt
import numpy as np
import math

from controller import Keyboard
from controller import Supervisor

from components import Pose, Map
from components import Manager, Localization, Mapping, SLAM, Device, RobotController, Manipulation



#Initialization
print("=== Initializing Grocery Shopper...")

m = Manager()

Localization    = Localization(m)
Mapping         = Mapping(m)
# Slam            = SLAM(m)
Device          = Device(m)
RobotController = RobotController(m)

m.Localization    = Localization
m.Mapping         = Mapping
# m.Slam            = SLAM
m.Device          = Device
m.RobotController = RobotController

# # Initialize the Webots Supervisor.
# supervisor = Supervisor()
# timeStep = Device.robot_step()

# Maniplation = Manipulation(4, supervisor, timeStep)




# planner = Planning()
# K = 1000 # Feel free to adjust as desired
# map = np.load("../assets/filter_map.npy")
# starting_point = [20,200]
# goal = np.array([325, 325])
# nodes = planner.rrt(starting_point, goal, K, 10, map)
# m.RobotController.set_waypoints(nodes)


m.RobotController.set_waypoints([
    (  4.80,  0.00,  0.00),
    (  4.79, -2.13,  0.00),
    ( -3.64, -2.03,  0.00),
    ( -3.44, -3.04,  1.57),
    (-12.53, -1.98,  0.00),
    (-12.10,  1.50,  3.14),
    (  4.91,  1.43,  0.00), #wut
    (  4.50,  5.57,  0.00),
    (-12.91,  5.55,  0.00),
    (-12.44, -5.44, -3.14),
    (  5.31, -5.69, -1.56),
    ])


gripper_status="closed"


####################################
####################################
def calc_input():
    v = 1.0  # [m/s]
    yawrate = 0.1  # [rad/s]
    u = np.array([[v, yawrate]]).T
    return u

# RFID positions [x, y]
RFID = np.array([
    [10.0, -2.0],
    [15.0, 10.0],
    [3.0, 15.0],
    [-5.0, 20.0]])
    
# State Vector [x y yaw v]'
STATE_SIZE = 0
xEst = np.zeros((STATE_SIZE, 1))
xTrue = np.zeros((STATE_SIZE, 1))
PEst = np.eye(STATE_SIZE)

xDR = np.zeros((STATE_SIZE, 1))  # Dead reckoning

# history
hxEst = xEst
hxTrue = xTrue
hxDR = xTrue

show_animation = True
####################################
####################################


# Main Loop
while Device.robot_step() != -1:
    vL, vR = RobotController.controller(
        control_type='auto', 
        vel_ratio=0.7,
        debug=False
    )
    pose = Localization.get_pose()
    
    point_cloud = Mapping.get_lidar_point_cloud(pose)
    Mapping.display_point_cloud(point_cloud)

    # image = Device.get_camera_image()

    Localization.update_odometry(vL, vR, print_pose=False)
    Device.set_wheel_joint_vel(vL, vR)






    ## Temporary commented code
    # if(gripper_status=="open"):
    #     # Close gripper, note that this takes multiple time steps...
    #     robot_parts["gripper_left_finger_joint"].setPosition(0)
    #     robot_parts["gripper_right_finger_joint"].setPosition(0)
    #     if right_gripper_enc.getValue()<=0.005:
    #         gripper_status="closed"
    # else:
    #     # Open gripper
    #     robot_parts["gripper_left_finger_joint"].setPosition(0.045)
    #     robot_parts["gripper_right_finger_joint"].setPosition(0.045)
    #     if left_gripper_enc.getValue()>=0.044:
    #         gripper_status="open"
