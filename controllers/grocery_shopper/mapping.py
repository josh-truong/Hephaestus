import numpy as np
import math

class Mapping:
    def __init__(self, MAX_SPEED, LIDAR_SENSOR_MAX_RANGE, LIDAR_ANGLE_BINS, LIDAR_ANGLE_RANGE, lidar_offsets):
        self.MAX_SPEED              = MAX_SPEED
        self.LIDAR_SENSOR_MAX_RANGE = LIDAR_SENSOR_MAX_RANGE
        self.LIDAR_ANGLE_BINS       = LIDAR_ANGLE_BINS
        self.LIDAR_ANGLE_RANGE      = LIDAR_ANGLE_RANGE
        self.lidar_offsets          = lidar_offsets

        self.map = np.zeros((360, 360))
        self.robot_poses = [] # List to hold robot previous poses

    def manual_control(self, keyboard, tol=0.5):
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
            # Part 1.4: Filter map and save to filesystem
            map = (self.map > tol).astype(int)
            np.save("map.npy",map)
            print("Map file saved")
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
    
    def homogenous_transform(self, rho, pose_x, pose_y, pose_theta):
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
            [ np.cos(pose_theta), -np.sin(pose_theta), pose_x],
            [ np.sin(pose_theta),  np.cos(pose_theta), pose_y],
            [0, 0, 1]
        ])
        # point_world_frame: m x 4 matrix
        point_world_frame = (T @ point_local_frame).T

        return point_world_frame[:, :2]

    def get_lidar_point_cloud(self, lidar, pose_x, pose_y, pose_theta):
        self.robot_poses.append(self.get_display_coords(pose_x, pose_y))

        lidar_sensor_readings = self.get_lidar_readings(lidar)
        point_cloud = self.homogenous_transform(lidar_sensor_readings, pose_x, pose_y, pose_theta)
        return point_cloud


    def display_point_cloud(self, display, point_cloud):
        for x, y in point_cloud:
            x, y = self.get_display_coords(x, y)
            pixel = np.clip(self.map[y][x] + 5e-3, 0.0, 1.0)
            self.map[y][x] = pixel

            grayscale = int(pixel*255)
            color = int(grayscale*256**2 + grayscale*256 + grayscale)

            display.setColor(color)
            display.drawPixel(x, y)

        display.setColor(0xFF0000)
        rx, ry = self.robot_poses[-1]
        display.drawPixel(rx, ry)