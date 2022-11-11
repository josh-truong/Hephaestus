import math

class Localization:

    def __init__(self, MAX_SPEED, MAX_SPEED_MS, AXLE_LENGTH):
        self.MAX_SPEED    = MAX_SPEED
        self.MAX_SPEED_MS = MAX_SPEED_MS
        self.AXLE_LENGTH  = AXLE_LENGTH

        self.pose_x     = 0
        self.pose_y     = 0
        self.pose_theta = 0

        self.vL = 0
        self.vR = 0

    def get_pose(self, gps, compass):
        self.pose_x = -gps.getValues()[0]
        self.pose_y = -gps.getValues()[1]

        n = compass.getValues()
        rad = ((math.atan2(n[0], n[1])))
        self.pose_theta = rad

        return self.pose_x, self.pose_y, self.pose_theta

    def update_pose(self, vL, vR, ts, print_pose=False):
        MAX_SPEED    = self.MAX_SPEED
        MAX_SPEED_MS = self.MAX_SPEED_MS
        AXLE_LENGTH  = self.AXLE_LENGTH
        pose_theta   = self.pose_theta

        self.pose_x += (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*ts/1000.0*math.cos(pose_theta)
        self.pose_y -= (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*ts/1000.0*math.sin(pose_theta)
        self.pose_theta += (vR-vL)/AXLE_LENGTH/MAX_SPEED*MAX_SPEED_MS*ts/1000.0

        if (print_pose):
            print(f"X: {self.pose_x:.2f} Z: {self.pose_y:.2f} Theta: {self.pose_theta:2f}")