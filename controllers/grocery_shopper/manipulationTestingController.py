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

# # Initialize the Webots Supervisor.
supervisor = Supervisor()
timeStep = Device.robot_step()

Maniplation = Manipulation(4, supervisor, timeStep)


# Main Loop
while Device.robot_step() != -1:
    pass