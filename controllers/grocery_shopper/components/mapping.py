import numpy as np
import math
from .utils import Map

class Mapping:
    def __init__(self, RobotConst, LidarConst):
        self.MAX_SPEED              = RobotConst.MAX_SPEED
        self.LIDAR_SENSOR_MAX_RANGE = LidarConst.SENSOR_MAX_RANGE
        self.LIDAR_ANGLE_BINS       = LidarConst.ANGLE_BINS
        self.LIDAR_ANGLE_RANGE      = LidarConst.ANGLE_RANGE
        self.lidar_offsets          = LidarConst.OFFSETS

        self.Map = Map()
        self.robot_poses = [] # List to hold robot previous poses

    def manual_control(self, keyboard, display, tol=0.5):
        MAX_SPEED = self.MAX_SPEED
        key = keyboard.getKey()
        
        vL, vR = 0, 0
        while(keyboard.getKey() != -1): pass
        if key == keyboard.LEFT :
            vL, vR = -MAX_SPEED, MAX_SPEED
        elif key == keyboard.RIGHT:
            vL, vR = MAX_SPEED, -MAX_SPEED
        elif key == keyboard.UP:
            vL, vR = MAX_SPEED, MAX_SPEED
        elif key == keyboard.DOWN:
            vL, vR = -MAX_SPEED, -MAX_SPEED
        elif key == ord(' '):
            vL, vR = 0, 0
        elif key == ord('S'):
            self.Map.save()
        elif key == ord('L'):
            map = self.Map.load()
            self.display_point_cloud(display, None, redraw=True)
        elif key == ord('D'):
            self.Map.display()
        elif key == ord('F'):
            map = self.Map.filter(tol=0.5)
            self.Map.display(map)
        return vL, vR

    def get_display_coords(self, x, y, display=(360, 360), world=(30, 15)):
        x = (display[0]*0.5) - (x * (display[0]/world[0]))
        y = display[1] - ((display[1]*0.5) - (y * (display[1]/world[1])))
        x, y = np.clip(x, 0, display[0]-1), np.clip(y, 0, display[1]-1)
        return int(x), int(y)

    def get_lidar_readings(self, lidar):
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

        # point_local_frame: 4 x m matrix
        point_local_frame = np.array([
            -np.cos(self.lidar_offsets) * rho + 0.202,
             np.sin(self.lidar_offsets) * rho - 0.004,
            np.ones(len(rho))
        ])
        # Drop any columns that contains inf
        point_local_frame = point_local_frame[:, ~np.isinf(point_local_frame).any(axis=0)]
        # T: 4 x 4 matrix
        T = np.array([
            [ np.cos(pose.theta), -np.sin(pose.theta), pose.x],
            [ np.sin(pose.theta),  np.cos(pose.theta), pose.y],
            [0, 0, 1]
        ])
        # point_world_frame: m x 4 matrix
        point_world_frame = (T @ point_local_frame).T
        return point_world_frame

    def get_lidar_point_cloud(self, lidar, pose):
        self.robot_poses.append(self.get_display_coords(pose.x, pose.y))

        lidar_sensor_readings = self.get_lidar_readings(lidar)
        point_cloud = self.homogenous_transform(lidar_sensor_readings, pose)
        return point_cloud[:, :2]


    def display_point_cloud(self, display, point_cloud, redraw=False):
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