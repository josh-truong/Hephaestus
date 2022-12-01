"""
mapping.py

Created on Fri Nov 19 2022
@Lead: Joshua Truong
"""

import numpy as np
import math
from .utils import Map


class Mapping:
    def __init__(self, m):
        print("=== Mapping Component Initialized...")
        self.m = m
        self.Map = Map()
        self.robot_poses = [] # List to hold robot previous poses

    def get_display_coords(self, x, y, display=(360, 360), world=(30, 15)):
        x = (display[0]*0.5) - (x * (display[0]/world[0]))
        y = display[1] - ((display[1]*0.5) - (y * (display[1]/world[1])))
        x, y = np.clip(x, 0, display[0]-1), np.clip(y, 0, display[1]-1)
        return int(x), int(y)

    def get_lidar_readings(self):
        lidar = self.m.Device.lidar
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
        lConst = self.m.lConst
        lidar_offsets = lConst.OFFSETS
        LIDAR_SENSOR_MAX_RANGE = lConst.SENSOR_MAX_RANGE

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

    def get_lidar_point_cloud(self, pose):
        self.robot_poses.append(self.get_display_coords(pose.x, pose.y))

        lidar_sensor_readings = self.get_lidar_readings()
        point_cloud = self.homogenous_transform(lidar_sensor_readings, pose)
        return point_cloud[:, :2]


    def display_point_cloud(self, point_cloud, redraw=False):
        display = self.m.Device.display
        if (redraw):
            for i in range(self.Map.shape[0]*self.Map.shape[1]):
                y, x = i // self.Map.shape[1], i % self.Map.shape[1]

                display.setColor(0x000000)
                display.drawPixel(x, y)

                grayscale = int(self.Map.pixel(x, y)*255)
                color = int(grayscale*256**2 + grayscale*256 + grayscale)
                display.setColor(color)
                display.drawPixel(x, y)

        if (point_cloud is None): return
        for x, y in point_cloud:
            x, y = self.get_display_coords(x, y)
            pixel = np.clip(self.Map.pixel(x, y) + 5e-3, 0.0, 1.0)
            self.Map.update_pixel(x, y, pixel)

            grayscale = int(pixel*255)
            color = int(grayscale*256**2 + grayscale*256 + grayscale)

            display.setColor(color)
            display.drawPixel(x, y)

        display.setColor(0xFF0000)
        rx, ry = self.robot_poses[-1]
        display.drawPixel(rx, ry)