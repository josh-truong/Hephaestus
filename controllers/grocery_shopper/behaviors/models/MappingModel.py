"""
MappingModel.py

Last updated on Fri Dec 9 2022
@Lead: Joshua Truong
"""

import numpy as np
import math
from .utils import Map
import imageio
import matplotlib.pyplot as plt

class MappingModel:
    def __init__(self, writer, reader):
        print("=== Mapping Component Initialized...")
        self.w, self.r = writer, reader
        self.robot_poses = [] # List to hold robot previous poses
        self.variances = [0]

    def get_lidar_readings(self):
        lidar = self.r.device.lidar
        lidar_sensor_readings = lidar.getRangeImage()
        lidar_sensor_readings = lidar_sensor_readings[83:len(lidar_sensor_readings)-83]
        return lidar_sensor_readings
    
    def homogenous_transform(self, rho, pose):
        """
        homogenous_transform method converts from robot into world coordinates.
        This method uses matrix operations, instead of the expanded form in a loop.

        # Expanded equivalence for one point
        rx = -math.cos(alpha)*rho + 0.202
        ry =  math.sin(alpha)*rho - 0.004

        # Convert detection from robot coordinates into world coordinates
        wx =  math.cos(pose_theta)*rx - math.sin(pose_theta)*ry + pose_x
        wy =  +(math.sin(pose_theta)*rx + math.cos(pose_theta)*ry) + pose_y
        """
        lidar_offsets = self.r.constants.lidar.OFFSETS
        LIDAR_SENSOR_MAX_RANGE = self.r.constants.lidar.SENSOR_MAX_RANGE

        # point_local_frame: 3 x m matrix
        point_local_frame = np.array([
            -np.cos(lidar_offsets) * rho + 0.202,
             np.sin(lidar_offsets) * rho - 0.004,
            np.ones(len(rho))
        ])
        # Drop any columns that exceed LIDAR_SENSOR_MAX_RANGE
        within_range = np.where(np.array(rho) <= LIDAR_SENSOR_MAX_RANGE)[0]
        point_local_frame = point_local_frame[:, within_range]

        # Drop any columns that contains inf
        point_local_frame = point_local_frame[:, ~np.isinf(point_local_frame).any(axis=0)]
        
        # T: 3 x 3 matrix
        T = np.array([
            [ np.cos(pose.theta), -np.sin(pose.theta), pose.x],
            [ np.sin(pose.theta),  np.cos(pose.theta), pose.y],
            [0, 0, 1]
        ])
        # point_world_frame: m x 3 matrix
        point_world_frame = (T @ point_local_frame).T
        return point_world_frame

    def get_lidar_point_cloud(self, pose, lidar_sensor_readings=None):
        if (lidar_sensor_readings is None):
            lidar_sensor_readings = self.get_lidar_readings()
        point_cloud = self.homogenous_transform(lidar_sensor_readings, pose)
        return point_cloud[:, :2]