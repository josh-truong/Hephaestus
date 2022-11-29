"""grocery controller."""
import matplotlib.pyplot as plt
import numpy as np
import math

from controller import Keyboard

from components import Pose, Map
from components import Manager, Localization, Mapping, SLAM, Device, RobotController



#Initialization
print("=== Initializing Grocery Shopper...")

m = Manager()

Localization    = Localization(m)
Mapping         = Mapping(m)
Slam            = SLAM(m)
Device          = Device(m)
RobotController = RobotController(m)

m.Localization    = Localization
m.Mapping         = Mapping
m.Slam            = SLAM
m.Device          = Device
m.RobotController = RobotController

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
        vel_ratio=0.5,
        debug=False
    )
    pose = Localization.get_pose()
    
    point_cloud = Mapping.get_lidar_point_cloud(pose)
    Mapping.display_point_cloud(point_cloud)

    # image = Device.get_camera_image()

    Localization.update_odometry(vL, vR, print_pose=False)
    Device.set_wheel_joint_vel(vL, vR)


    # slam.ekf(xEst, PEst, u, z)


    # u = calc_input()
    # xTrue, z, xDR, ud = slam.observation(xTrue, xDR, u, RFID)
    # xEst, PEst = ekf_slam(xEst, PEst, ud, z)
    # x_state = xEst[0:STATE_SIZE]
    # # store data history
    # hxEst = np.hstack((hxEst, x_state))
    # hxDR = np.hstack((hxDR, xDR))
    # hxTrue = np.hstack((hxTrue, xTrue))
    # if show_animation:  # pragma: no cover
    #         plt.cla()
    #         plt.plot(RFID[:, 0], RFID[:, 1], "*k")
    #         plt.plot(xEst[0], xEst[1], ".r")
    #         # plot landmark
    #         for i in range(calc_n_LM(xEst)):
    #             plt.plot(xEst[STATE_SIZE + i * 2],
    #                      xEst[STATE_SIZE + i * 2 + 1], "xg")
    #         plt.plot(hxTrue[0, :],
    #                  hxTrue[1, :], "-b")
    #         plt.plot(hxDR[0, :],
    #                  hxDR[1, :], "-k")
    #         plt.plot(hxEst[0, :],
    #                  hxEst[1, :], "-r")
    #         plt.axis("equal")
    #         plt.grid(True)
    #         plt.pause(0.001)



    





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
