"""grocery controller."""
import matplotlib.pyplot as plt
import numpy as np
import math

from controller import Keyboard
from controller import Supervisor

from components import Pose, Map
from components import Manager, Localization, Mapping, SLAM, Device, RobotController, Manipulation, Planning, EdgeDetection



#Initialization
print("=== Initializing Grocery Shopper...")

m = Manager()

Localization    = Localization(m)
Mapping         = Mapping(m)
# Slam            = SLAM(m)
Device          = Device(m)
RobotController = RobotController(m)
planner         = Planning()
EdgeDetection   = EdgeDetection(m)


m.Localization    = Localization
m.Mapping         = Mapping
# m.Slam            = SLAM
m.Device          = Device
m.RobotController = RobotController
m.planner         = Planning
m.edgeDetection   = EdgeDetection

# # Initialize the Webots Supervisor.
# supervisor = Supervisor()
# timeStep = Device.robot_step()

# Maniplation = Manipulation(4, supervisor, timeStep)





# map = np.load("assets/filter_map.npy")
# m.Device.redraw_display(map)


# nodes = planner.rrt(starting_point=[122, 180], goal_point=np.array([333, 339]), k=1000, delta_q=10, map=map)
## planner.visualize_2D_graph(map, nodes, np.array([333, 339]))

# waypoints = np.array(planner.getWaypoints(nodes, np.array([325, 325])))
# m.RobotController.set_waypoints(waypoints)
# for x, y, _ in waypoints:
#     x, y = m.Mapping.get_display_coords(x, y)
#     m.Device.display.setColor(0x00FF00)
#     m.Device.display.drawPixel(x, y)


# np.save('path.npy', [Mapping.get_display_coords(x,y) for x,y,_ in waypoints])

# pt1, pt2 = (4.80, 0.00), (-13.02, -5.42)
# pt1 = Mapping.get_display_coords(pt1[0], pt1[1])
# pt2 = Mapping.get_display_coords(pt2[0], pt2[1])
# print(f"Start {pt1}")
# print(f"End: {pt2}")
# print("")
# print(f"Start {planner.get_world_coords(pt1[0], pt1[1])}")
# print(f"End: {planner.get_world_coords(pt2[0], pt2[1])}")


m.RobotController.set_waypoints([
    (  4.80,  0.00,  0.00),
    (  4.79, -2.22,  0.00),
    (-12.83, -2.30,  0.00),
    (-12.78,  2.03,  0.00),
    (  4.89,  1.80,  0.00),
    (  4.94,  5.68,  0.00),
    (-13.97,  5.93,  0.00),
    (-13.24, -5.64,  0.00),
    (  5.51, -5.89,  0.00),
    ( 13.30, -6.76,  0.00),
    ( 13.36, -3.97,  0.00),
    ( 12.49, -3.95,  0.00),
    ( 13.36, -3.97,  0.00),
    ( 12.69, -3.47,  0.00),
    ( 13.48,  6.47,  0.00),
    (  4.79,  5.68,  0.00),
    ( 12.69, -3.47,  0.00),
    ( 13.36, -3.97,  0.00),
    ( 12.49, -3.95,  0.00),
    ( 13.36, -3.97,  0.00),
    ( 13.30, -6.76,  0.00),
    (  5.51, -5.89,  0.00),
    (-13.24, -5.64,  0.00),
    (-13.97,  5.93,  0.00),
    (  4.94,  5.68,  0.00),
    (  4.89,  1.80,  0.00),
    (-12.78,  2.03,  0.00),
    (-12.83, -2.30,  0.00),
    (  4.79, -2.22,  0.00),
    (  4.80,  0.00,  0.00),
    ])


gripper_status="closed"


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
    EdgeDetection.run()


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
