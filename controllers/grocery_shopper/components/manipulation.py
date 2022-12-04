"""
manipulation.py

Created on Tues Nov 29 2022
@Lead: Nathan Kochera
"""
import numpy as np
import math
import ikpy
from ikpy.chain import Chain

"""
Used the following github instructions to learn the basics of the ikpy library
https://gist.github.com/ItsMichal/4a8fcb330d04f2ccba582286344dd9a7
"""

class Manipulation:

    def __init__(self, maxIterations, supervisor, timeStep):
        self.supervisor = supervisor
        self.timeStep = timeStep
        self.chain = Chain.from_urdf_file("robot_urdf.urdf", last_link_vector=[0.004, 0,-0.1741], base_elements=["base_link", "base_link_Torso_joint", "Torso", "torso_lift_joint", "torso_lift_link", "torso_lift_link_TIAGo front arm_11367_joint", "TIAGo front arm_11367"])
        print(self.chain.links)

        # The Tiago robot has multiple motors, each identified by their names below.
        # Make sure to use a parts list specific to your robot's controllable joints
        self.part_names = ("head_2_joint", "head_1_joint", "torso_lift_joint", "arm_1_joint",
                    "arm_2_joint",  "arm_3_joint",  "arm_4_joint",      "arm_5_joint",
                    "arm_6_joint",  "arm_7_joint",  "wheel_left_joint", "wheel_right_joint")

        for link_id in range(len(self.chain.links)):

            # link object
            link = self.chain.links[link_id]
            
            # disabling joints we can't/don't want to use
            if link.name not in self.part_names or  link.name =="torso_lift_joint":
                print("Disabling {}".format(link.name))
                self.chain.active_links_mask[link_id] = False

        # Initialize the motors
        self.motors = []
        for link in self.chain.links:
            if link.name in self.part_names and link.name != "torso_lift_joint":
                motor = supervisor.getDevice(link.name)

                # Account for varying max velocities
                if link.name == "torso_lift_joint":
                    motor.setVelocity(0.07)
                else:
                    motor.setVelocity(1)
                    
                position_sensor = motor.getPositionSensor()
                position_sensor.enable(timeStep)
                self.motors.append(motor)

    def getInitialPosition(self):
        initial_position = [0,0,0,0] + [m.getPositionSensor().getValue() for m in self.motors] + [0,0,0,0]
        return initial_position

    def applyIKResults(self, results):
        for res in range(len(results)):
            # ignore motors that can't be controlled
            if (self.chain.links[res].name in self.part_names):
                self.supervisor.getDevice(self.chain.links[res].name).setPosition(results[res])
                print("Setting {} to {}".format(self.chain.links[res].name, results[res]))

    def moveArmToBox(self):
        target = [0.30,0,0.70]
        ikResults = self.chain.inverse_kinematics(target, initial_position=self.getInitialPosition(),  target_orientation = [0,0,1], orientation_mode="Y")
        self.applyIKResults(ikResults)

    def setDrivingState(self):
        target = [0,0,7]
        ikResults = self.chain.inverse_kinematics(target, initial_position=self.getInitialPosition(),  target_orientation = [0,0,1], orientation_mode="Y")
        self.applyIKResults(ikResults)

    def openGripper(self):
        self.supervisor.getDevice("gripper_right_finger_joint").setPosition(0.045)
        self.supervisor.getDevice("gripper_left_finger_joint").setPosition(0.045)
                

    def closeGripper(self):
        self.supervisor.getDevice("gripper_right_finger_joint").setPosition(0.025)
        self.supervisor.getDevice("gripper_left_finger_joint").setPosition(0.025)

    def grabCube(self):
        target = [1.0,0.075,1.10]
        ikResults = self.chain.inverse_kinematics(target, initial_position=self.getInitialPosition(),  target_orientation = [0,0,1], orientation_mode="Y")
        self.applyIKResults(ikResults)

    def moveArmToCube(self):
        # need to get cube position
        target = [1.0,0.075,1.25]
        ikResults = self.chain.inverse_kinematics(target, initial_position=self.getInitialPosition(),  target_orientation = [0,0,1], orientation_mode="Y")
        self.applyIKResults(ikResults)
        # self.grabCube(target)


