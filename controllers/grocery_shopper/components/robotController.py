"""
controller.py

Created on Fri Nov 19 2022
@Lead: Joshua Truong
"""

class RobotController:
    def __init__(self, m):
        self.m = m
        self.vL, self.vR = 0, 0

    def controller(self, control_type='manual', vel_ratio=1):
        keyboard = self.m.Device.keyboard
        key = keyboard.getKey()
        while(keyboard.getKey() != -1): pass

        self.display_commands(key)

        vL, vR = 0, 0
        if (control_type == 'manual'):
            vL, vR = self.manual(key)
        elif (control_type == 'ik'):
            vL, vR = self.inverse_kinematics()
        vL, vR = vL*vel_ratio, vR*vel_ratio
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

    def inverse_kinematics(self):
        raise NotImplementedError()

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