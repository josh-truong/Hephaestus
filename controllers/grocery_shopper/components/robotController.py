"""
controller.py

Created on Fri Nov 19 2022
@Lead: Joshua Truong
"""
import math
import numpy as np
from .utils import Pose

class RobotController:
    def __init__(self, m, waypoints=None, rho_tol=0.2, alpha_tol=math.pi/10, eta_tol=math.pi/10):
        self.m = m
        self.vL, self.vR = 0, 0

        self.state = 0
        self.waypoints = waypoints

        self.rtol, self.atol, self.etol = rho_tol, alpha_tol, eta_tol
        self.flex_rtol, self.flex_atol = rho_tol, alpha_tol

    def controller(self, control_type='man', vel_ratio=1, debug=False):
        keyboard = self.m.Device.keyboard
        key = keyboard.getKey()
        while(keyboard.getKey() != -1): pass

        self.display_commands(key)

        vL, vR = 0, 0
        if (control_type == 'man'):
            vL, vR = self.manual(key)
            vL, vR = vL*vel_ratio, vR*vel_ratio
        elif (control_type == 'auto'):
            vL, vR = self.autonomous(debug, vel_ratio)
        self.obstacle_avoidance()
        self.vL, self.vR = vL, vR
        return vL, vR

    def manual(self, key):
        keyboard = self.m.Device.keyboard
        MAX_SPEED = self.m.rConst.MAX_SPEED
        vL, vR = 0, 0
        if key == keyboard.LEFT :
            vL, vR = -MAX_SPEED,  MAX_SPEED
        elif key == keyboard.RIGHT:
            vL, vR =  MAX_SPEED, -MAX_SPEED
        elif key == keyboard.UP:
            vL, vR =  MAX_SPEED,  MAX_SPEED
        elif key == keyboard.DOWN:
            vL, vR = -MAX_SPEED, -MAX_SPEED
        return vL, vR

    def autonomous(self, debug, vRatio):
        """Uses inverse_kinematics and a set of waypoints generated by path planning algorithm"""
        if (self.waypoints is None):
            raise ValueError("waypoints cannot be none in inverse_kinematics!")
        goal = Pose(self.waypoints[self.state])

        # Calculate Error
        rho   = self.m.Localization.get_position_error(goal)
        alpha = self.m.Localization.get_bearing_error(goal)
        eta   = self.m.Localization.get_heading_error(goal)

        # Feedback Controller
        # Controller gains
        p1 = 0.4 # dist
        p2 = 0.7 # bearing
        p3 = 0.05 # heading
        x_dot, theta_dot = 0, 0
        if (rho < self.flex_rtol):
            self.flex_rtol += 0.01
            theta_dot = p3*eta
        elif (abs(alpha) > self.flex_atol):
            self.flex_atol += 0.001
            theta_dot = p2*alpha
        else:
            x_dot = p1*rho
            theta_dot = p2*alpha + p3*eta
        
        # Adjust wheelspeeds
        AXLE_LENGTH = self.m.rConst.AXLE_LENGTH
        MAX_SPEED = self.m.rConst.MAX_SPEED

        phi_l = x_dot - ((theta_dot * AXLE_LENGTH) / 2) # Left wheel velocity in rad/s
        phi_r = x_dot + ((theta_dot * AXLE_LENGTH) / 2) # Right wheel velocity in rad/s
        
        # Normalize wheelspeed
        turn_ratio  = 0.20 if (np.sign(phi_l) != np.sign(phi_r)) else 1
        phi_l_ratio = 1 if (abs(phi_l) > abs(phi_r)) else abs(phi_l/phi_r)
        phi_r_ratio = 1 if (abs(phi_r) > abs(phi_l)) else abs(phi_r/phi_l)
        phi_l = np.sign(phi_l)*MAX_SPEED*vRatio*phi_l_ratio*turn_ratio
        phi_r = np.sign(phi_r)*MAX_SPEED*vRatio*phi_r_ratio*turn_ratio

        # Clamp wheel speeds
        def clamp(n, upper=MAX_SPEED, lower=-MAX_SPEED):
            return max(min(upper, n), lower)
        vL, vR = clamp(phi_l), clamp(phi_r)

        # Stopping criteria
        if (rho < self.flex_rtol and abs(eta) < self.etol):
            # Move to next waypoint criteria
            self.state += 1 if (self.state != len(self.waypoints)-1) else 0
            vL, vR = 0, 0
            # A method to prevent robot oscillating
            self.flex_rtol = self.rtol
            self.flex_atol = self.atol

        if (debug):
            pose = self.m.Localization.Pose
            print(f"Current State: {self.state:} Final State: {len(self.waypoints)-1}")
            print(f"start: ({pose.x:< 6.2f}, {pose.y:< 6.2f}, {pose.theta:< 6.2f})")
            print(f"goal:  ({goal.x:< 6}, {goal.y:< 6}, {goal.theta:< 6})")
            print(f"rho_tol: {self.flex_rtol:< 6.2f} alpha_tol: {self.flex_atol:< 6.2f} eta_tol: {self.etol:< 6.2f}")
            print(f"    rho: {rho:< 6.2f}     alpha: {alpha:< 6.2f}     eta: {eta:< 6.2f}")
            print(f"x_dot: {x_dot:< 6.2f} theta_dot: {theta_dot:< 6.2f}")
            print(F"vL: {vL:< 6.2f} vR: {vR:< 6.2f}")
            print("=========================")
        return vL, vR

    def obstacle_avoidance(self, dtol=0.86, sampling=20):
        while(self.m.Device.robot_step() != -1):
            lidar_sensor_readings = self.m.Mapping.get_lidar_readings()
            lDist = np.mean(lidar_sensor_readings[:sampling])
            rDist = np.mean(lidar_sensor_readings[-sampling:])

            if (lDist > dtol and rDist > dtol):
                break

            MAX_SPEED = self.m.rConst.MAX_SPEED
            vL, vR = (MAX_SPEED*0.2, MAX_SPEED*0.1) if (lDist < rDist) else (MAX_SPEED*0.1, MAX_SPEED*0.2)
            self.m.Localization.update_odometry(vL, vR, print_pose=False)
            self.m.Device.set_wheel_joint_vel(vL, vR)

    def set_waypoints(self, waypoints):
        self.waypoints = waypoints

    def get_wheel_velocity(self):
        return self.vL, self.vR

    def display_commands(self, key, filter_tol=0.5):
        map = self.m.Mapping.Map
        if key == ord('S'):
            map.save()
        elif key == ord('L'):
            map = map.load()
            self.m.Mapping.display_point_cloud(None, redraw=True)
        elif key == ord('D'):
            map.display()
        elif key == ord('F'):
            f_map = map.filter(tol=filter_tol)
            map.display(f_map)