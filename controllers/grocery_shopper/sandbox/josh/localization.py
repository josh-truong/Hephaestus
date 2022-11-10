import math

"""
    Localization - Tier 1
"""
class Localization:

    def __init__(self, gps, compass, pose_x=0, pose_y=0, pose_theta=0):
        self.gps     = gps
        self.compass = compass

        self.MAX_SPEED    = 7.0
        self.MAX_SPEED_MS = 0.633
        self.AXLE_LENGTH  = 0.4044

        self.pose_x     = pose_x
        self.pose_y     = pose_y
        self.pose_theta = pose_theta

    def get_pose(self):
        self.pose_y = -gps.getValues()[1]
        self.pose_x = -gps.getValues()[0]

        n = compass.getValues()
        rad = math.atan2(n[0], -n[2]) #-1.5708
        self.pose_theta = rad

        return (self.pose_x, self.pose_y, self.pose_theta)

    def update_pose(self, vL, vR, timestep, print_pose=False):
        MAX_SPEED    = self.MAX_SPEED
        MAX_SPEED_MS = self.MAX_SPEED_MS
        AXLE_LENGTH  = self.AXLE_LENGTH
        pose_theta   = self.pose_theta

        self.pose_x += (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.cos(pose_theta)
        self.pose_y -= (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.sin(pose_theta)
        self.pose_theta += (vR-vL)/AXLE_LENGTH/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0

        if (print_pose):
            print("X: %f Z: %f Theta: %f" % (pose_x, pose_y, pose_theta))