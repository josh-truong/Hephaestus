"""grocery controller."""
import matplotlib.pyplot as plt
import numpy as np


from controller import Keyboard

from components import Pose, Map
from components import Manager, Localization, Mapping, SLAM, Device, RobotController



#Initialization
print("=== Initializing Grocery Shopper...")

m = Manager()
m.Localization    = Localization(m)
m.Mapping         = Mapping(m)
m.Slam            = SLAM(m)
m.Device          = Device(m)
m.RobotController = RobotController(m)







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
STATE_SIZE = 3
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

goal = Pose(-13, -5)

# Main Loop
while m.Device.robot_step() != -1:
    vL, vR = m.RobotController.controller(control_type='manual', vel_ratio=1)
    pose = m.Localization.get_pose()
    
    point_cloud = m.Mapping.get_lidar_point_cloud(pose)
    m.Mapping.display_point_cloud(point_cloud)

    # rho   = localization.get_position_error(goal)
    # alpha = localization.get_bearing_error(goal)
    # eta   = localization.get_heading_error(goal)

    m.Localization.update_odometry(vL, vR, print_pose=False)
    m.Device.set_wheel_joint_vel(vL, vR)


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
