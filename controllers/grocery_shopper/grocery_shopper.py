"""grocery controller."""
import numpy as np
import math

from controller import Keyboard
from controller import Supervisor
from controller import Robot

from components import Pose, Map
from components import Manager, Localization, Mapping, SLAM, Device, RobotController, Manipulation, Planning, EdgeDetection, Vision



#Initialization
print("=== Initializing Grocery Shopper...")


supervisor = Supervisor()
timeStep = int(4*supervisor.getBasicTimeStep())
vision = Vision(supervisor, timeStep)
manipulation = Manipulation(4, supervisor, timeStep)

## Main Loop
count = 0
xTarget = 0
yTarget = 0
top = True
while supervisor.step(timeStep) != -1:
    if (count == 0):
        locations, blobs, mask, img = vision.detect()
        print(locations)
        # vision.show_image(img, mask, True)
        for blob in blobs:
            print(len(blob))
        diff = (120 - locations[0][1])
        if diff <= 0:
            xTarget = (120 - locations[0][1]) * 0.0088#0.009204136
        else:
            xTarget = (120 - locations[0][1]) * 0.0095#0.009204136
        if locations[0][0] <= 75:
            yTarget = 1.25
            top = True
        elif locations[0][0] <= 130:
            yTarget = 0.7
            top = False
        else:
            yTarget = 1.25

    if count == 0:
        if top:
            manipulation.setDrivingState()
        else:
            manipulation.setMidState()
    if count == 20:
        manipulation.openGripper()
    if count == 30:
        manipulation.moveArmToCube(xTarget, yTarget)
    if count == 70:
        manipulation.grabCube(xTarget, yTarget)
    if count == 80:
        manipulation.closeGripper()
    if count == 90:
        if top:
            manipulation.setDrivingState()
        else:
            manipulation.setMidState()
    if count == 110:
        manipulation.moveArmToBox()
    if count == 140:
        manipulation.openGripper()
    count += 1


# """grocery controller."""
# import numpy as np
# import math

# from controller import Keyboard
# from controller import Supervisor

# from components import Pose, Map
# from components import Manager, Localization, Mapping, SLAM, Device, RobotController, Manipulation, Planning, EdgeDetection



# #Initialization
# print("=== Initializing Grocery Shopper...")

# m = Manager()

# Localization    = Localization(m)
# Mapping         = Mapping(m)
# # Slam            = SLAM(m)
# Device          = Device(m)
# RobotController = RobotController(m)
# planner         = Planning()
# EdgeDetection   = EdgeDetection(m)


# m.Localization    = Localization
# m.Mapping         = Mapping
# # m.Slam            = SLAM
# m.Device          = Device
# m.RobotController = RobotController
# m.planner         = Planning
# m.edgeDetection   = EdgeDetection

# # # Initialize the Webots Supervisor.
# # supervisor = Supervisor()
# # timeStep = Device.robot_step()

# # Maniplation = Manipulation(4, supervisor, timeStep)
# supervisor = Supervisor()
# timeStep = int(4*supervisor.getBasicTimeStep())





# # map = np.load("assets/filter_map.npy")
# # m.Device.redraw_display(map)


# # nodes = planner.rrt(starting_point=[122, 180], goal_point=np.array([333, 339]), k=1000, delta_q=10, map=map)
# ## planner.visualize_2D_graph(map, nodes, np.array([333, 339]))

# # waypoints = np.array(planner.getWaypoints(nodes, np.array([325, 325])))
# # m.RobotController.set_waypoints(waypoints)
# # for x, y, _ in waypoints:
# #     x, y = m.Mapping.get_display_coords(x, y)
# #     m.Device.display.setColor(0x00FF00)
# #     m.Device.display.drawPixel(x, y)


# # np.save('path.npy', [Mapping.get_display_coords(x,y) for x,y,_ in waypoints])

# # pt1, pt2 = (4.80, 0.00), (-13.02, -5.42)
# # pt1 = Mapping.get_display_coords(pt1[0], pt1[1])
# # pt2 = Mapping.get_display_coords(pt2[0], pt2[1])
# # print(f"Start {pt1}")
# # print(f"End: {pt2}")
# # print("")
# # print(f"Start {planner.get_world_coords(pt1[0], pt1[1])}")
# # print(f"End: {planner.get_world_coords(pt2[0], pt2[1])}")
# # planner = Planning()
# ## K = 1000 # Feel free to adjust as desired
# ## map = np.load("../assets/filter_map.npy")
# ## starting_point = [20,200]
# ## goal = np.array([325, 325])
# ## nodes = planner.rrt(starting_point, goal, K, 10, map)
# ## m.RobotController.set_waypoints(nodes)


# m.RobotController.set_waypoints([
#     (  4.80,  0.00,  0.00), # First
#     (  4.79, -2.22,  0.00),
#     (-12.83, -2.30,  0.00),
#     (-12.78,  2.03,  0.00),
#     (  4.89,  1.80,  0.00),
#     (  4.94,  5.68,  0.00),
#     (-13.01,  5.51,  0.00),
#     (-13.24, -5.64,  0.00),
#     (  5.51, -5.89,  0.00), # Second
#     (  5.51, -5.89,  0.00),
#     (-13.24, -5.64,  0.00),
#     (-13.01,  5.51,  0.00),
#     (  4.94,  5.68,  0.00),
#     (  4.89,  1.80,  0.00),
#     (-12.78,  2.03,  0.00),
#     (-12.83, -2.30,  0.00),
#     (  4.79, -2.22,  0.00),
#     (  4.80,  0.00,  0.00), # Third
#     (  4.79, -2.22,  0.00),
#     (-12.83, -2.30,  0.00),
#     (-12.78,  2.03,  0.00),
#     (  4.89,  1.80,  0.00),
#     (  4.94,  5.68,  0.00),
#     (-13.01,  5.51,  0.00),
#     (-13.24, -5.64,  0.00),
#     (  5.51, -5.89,  0.00),

#     # ( 13.36, -3.97,  0.00),
#     # ( 12.49, -3.95,  0.00),
#     # ( 13.36, -3.97,  0.00),
#     # ( 12.69, -3.47,  0.00),
#     # ( 13.48,  6.47,  0.00),
#     # (  4.79,  5.68,  0.00),
#     # ( 12.69, -3.47,  0.00),
#     # ( 13.36, -3.97,  0.00),
#     # ( 12.49, -3.95,  0.00),
#     # ( 13.36, -3.97,  0.00),
#     # ( 13.30, -6.76,  0.00),
#     # (  5.51, -5.89,  0.00),
#     # (-13.24, -5.64,  0.00),
#     # (-13.97,  5.93,  0.00),
#     # (  4.94,  5.68,  0.00),
#     # (  4.89,  1.80,  0.00),
#     # (-12.78,  2.03,  0.00),
#     # (-12.83, -2.30,  0.00),
#     # (  4.79, -2.22,  0.00),
#     # (  4.80,  0.00,  0.00),
#     ])
# # m.RobotController.set_waypoints([
#     # (  4.80,  0.00,  0.00),
#     # (  4.79, -2.13,  0.00),
#     # ( -3.64, -2.03,  0.00),
#     # ( -3.44, -3.04,  1.57),
#     # (-12.53, -1.98,  0.00),
#     # (-12.10,  1.50,  3.14),
#     # (  4.91,  1.43,  0.00), #wut
#     # (  4.50,  5.57,  0.00),
#     # (-12.91,  5.55,  0.00),
#     # (-12.44, -5.44, -3.14),
#     # (  5.31, -5.69, -1.56),
#     # ])


# gripper_status="closed"

# start_bounding=0
# # Main Loop
# while Device.robot_step() != -1:
#     vL, vR = RobotController.controller(
#         control_type='auto', 
#         vel_ratio=0.6,
#         debug=True
#     )
#     pose = Localization.get_pose()
# # gripper_status="closed"


# # ####################################
# # ####################################
# # def calc_input():
#     # v = 1.0  # [m/s]
#     # yawrate = 0.1  # [rad/s]
#     # u = np.array([[v, yawrate]]).T
#     # return u

# ## RFID positions [x, y]
# # RFID = np.array([
#     # [10.0, -2.0],
#     # [15.0, 10.0],
#     # [3.0, 15.0],
#     # [-5.0, 20.0]])
    
# ## State Vector [x y yaw v]'
# # STATE_SIZE = 0
# # xEst = np.zeros((STATE_SIZE, 1))
# # xTrue = np.zeros((STATE_SIZE, 1))
# # PEst = np.eye(STATE_SIZE)

# # xDR = np.zeros((STATE_SIZE, 1))  # Dead reckoning

# ## history
# # hxEst = xEst
# # hxTrue = xTrue
# # hxDR = xTrue

# # show_animation = True
# # ####################################
# # ####################################


# ## Main Loop
# # while Device.robot_step() != -1:
#     # vL, vR = RobotController.controller(
#         # control_type='auto', 
#         # vel_ratio=0.7,
#         # debug=False
#     # )
#     # pose = Localization.get_pose()
    
#     point_cloud = Mapping.get_lidar_point_cloud(pose)
#     Mapping.display_point_cloud(point_cloud)
#     EdgeDetection.run()


#     # point_cloud = Mapping.get_lidar_point_cloud(pose)
#     # Mapping.display_point_cloud(point_cloud)

#     if start_bounding%200 == 0 and start_bounding!=0:
#         rectangle_bounds = EdgeDetection.get_obstacle_bound(display=True)
#         pass
#     start_bounding += 1
#     # image = Device.get_camera_image()
#     ## image = Device.get_camera_image()

#     # Localization.update_odometry(vL, vR, print_pose=False)
#     # Device.set_wheel_joint_vel(vL, vR)



    



#     ## Temporary commented code
#     # ## Temporary commented code
#     # if(gripper_status=="open"):
#     #     # Close gripper, note that this takes multiple time steps...
#     #     robot_parts["gripper_left_finger_joint"].setPosition(0)
#     #     robot_parts["gripper_right_finger_joint"].setPosition(0)
#     #     if right_gripper_enc.getValue()<=0.005:
#     #         gripper_status="closed"
#     # else:
#     #     # Open gripper
#     #     robot_parts["gripper_left_finger_joint"].setPosition(0.045)
#     #     robot_parts["gripper_right_finger_joint"].setPosition(0.045)
#     #     if left_gripper_enc.getValue()>=0.044:
#     #         gripper_status="open"