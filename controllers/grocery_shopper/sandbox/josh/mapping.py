import numpy as np
from pose import Pose

class Mapping:
    def __init__(self, MAX_SPEED):
        self.MAX_SPEED = MAX_SPEED

        self.vL = 0
        self.vR = 0

        self.LIDAR_SENSOR_MAX_RANGE = 3 # Meters
        self.LIDAR_ANGLE_BINS       = 21 # 21 Bins to cover the angular range of the lidar, centered at 10
        self.LIDAR_ANGLE_RANGE      = 1.5708 # 90 degrees, 1.5708 radians
        self.lidar_sensor_readings  = []

        self.lidar_offsets = np.linspace(-self.LIDAR_ANGLE_RANGE*0.5, self.LIDAR_ANGLE_RANGE*0.5, self.LIDAR_ANGLE_BINS)
        self.lidar_offsets -= np.pi/2 # offset for lidar offset

    def controller(self, keyboard):
        key = keyboard.getKey()
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
            map = (map > .5).astype(int)
            np.save("map.npy",map)
            print("Map file saved")
        else: # slow down
            vL *= 0.75
            vR *= 0.75
        self.vL, self.vR = vL, vR
        return vL, vR

    def get_display_coords(self, x, y, scale=300):
        return [int(x*scale), int(y*scale)]

    def homogenous_transform(self, lidar, pose: Pose, display_scale=300):
        lidar_sensor_readings = lidar.getRangeImage()

        # point_local_frame: 4 x m matrix
        point_local_frame = np.array([
            self.lidar_sensor_readings * np.cos(self.lidar_offsets),
            self.lidar_sensor_readings * np.sin(self.lidar_offsets),
            np.zeros(self.LIDAR_ANGLE_BINS),
            np.ones(self.LIDAR_ANGLE_BINS)
        ])
        # Drop any columns that contains inf
        point_local_frame = point_local_frame[:, ~np.isinf(point_local_frame).any(axis=0)]
        
        # T: 4 x 4 matrix
        T = np.array([
            [ np.cos(pose.theta), np.sin(pose.theta), 0, pose.x],
            [-np.sin(pose.theta), np.cos(pose.theta), 0, pose.y],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        
        # point_world_frame: m x 4 matrix
        point_world_frame = (T @ point_local_frame).T

        return point_world_frame