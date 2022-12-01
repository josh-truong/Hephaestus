"""grocery controller."""
import matplotlib.pyplot as plt
import numpy as np
import math

from controller import Keyboard
from controller import Supervisor

from components import Pose, Map
from components import Manager, Localization, Mapping, SLAM, Device, RobotController, Manipulation, Planning



#Initialization
print("=== Initializing Grocery Shopper...")

m = Manager()

Localization    = Localization(m)
Mapping         = Mapping(m)
# Slam            = SLAM(m)
Device          = Device(m)
RobotController = RobotController(m)
planner         = Planning()

m.Localization    = Localization
m.Mapping         = Mapping
# m.Slam            = SLAM
m.Device          = Device
m.RobotController = RobotController
m.planner         = Planning

# # Initialize the Webots Supervisor.
# supervisor = Supervisor()
# timeStep = Device.robot_step()

# Maniplation = Manipulation(4, supervisor, timeStep)





map = np.load("assets/filter_map.npy")
m.Mapping.Map.map = map
m.Mapping.display_point_cloud(None, redraw=True)

nodes = planner.rrt(starting_point=[122, 180], goal_point=np.array([333, 339]), k=1000, delta_q=10, map=map)
# planner.visualize_2D_graph(map, nodes, np.array([333, 339]))
waypoints = np.array(planner.getWaypoints(nodes, np.array([325, 325])))
m.RobotController.set_waypoints(waypoints)
print(waypoints[0], waypoints[1])
for x, y, _ in waypoints:
    x, y = m.Mapping.get_display_coords(x, y)
    m.Device.display.setColor(0x00FF00)
    m.Device.display.drawPixel(x, y)


# np.save('path.npy', [Mapping.get_display_coords(x,y) for x,y,_ in waypoints])

# pt1, pt2 = (4.80, 0.00), (-13.02, -5.42)
# pt1 = Mapping.get_display_coords(pt1[0], pt1[1])
# pt2 = Mapping.get_display_coords(pt2[0], pt2[1])
# print(f"Start {pt1}")
# print(f"End: {pt2}")
# print("")
# print(f"Start {planner.get_world_coords(pt1[0], pt1[1])}")
# print(f"End: {planner.get_world_coords(pt2[0], pt2[1])}")


# m.RobotController.set_waypoints([
#     (  4.80,  0.00,  0.00),
#     (  4.79, -2.13,  0.00),
#     ( -3.64, -2.03,  0.00),
#     ( -3.44, -3.04,  1.57),
#     (-12.53, -1.98,  0.00),
#     (-12.10,  1.50,  3.14),
#     (  4.91,  1.43,  0.00), #wut
#     (  4.50,  5.57,  0.00),
#     (-12.91,  5.55,  0.00),
#     (-12.44, -5.44, -3.14),
#     (  5.31, -5.69, -1.56),
#     ])


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
