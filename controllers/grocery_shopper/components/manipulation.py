import numpy as np
import math
import ikpy
from ikpy.chain import Chain


class Manipulation:

    def __init__(self, maxIterations, supervisor, timeStep):
        self.supervisor = supervisor
        self.timeStep = timeStep
        self.chain = Chain.from_urdf_file("controllers/grocery_shopper/robot_urdf.urdf")
        self.maxIterations = maxIterations
        self.motors = []
        for link in self.chain.links:
            if 'motor' in link.name:
                motor = self.supervisor.getDevice(link.name)
                motor.setVelocity(1.0)
                position_sensor = motor.getPositionSensor()
                position_sensor.enable(timeStep)
                self.motors.append(motor)

    # def setDrivingState():

