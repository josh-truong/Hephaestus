from controller import Robot
from .constants import RobotConst
from .mapping import Mapping as MappingClass


class Device:
    def __init__(self):
        print("=== Device Component Initialized...")
        self.robot_parts = {}

        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        self.keyboard = self.enable_keyboard()
        self.display  = self.enable_display()

        self.rConst = RobotConst()

    def robot_step(self):
        return self.robot.step(self.timestep)

    def get_timestep(self):
        return int(self.robot.getBasicTimeStep())

    def set_robot_parts(self, target_pos=None):
        """
        All motors except the wheels are controlled by position control. The wheels
        are controlled by a velocity controller. We therefore set their position to infinite.
        """
        if (target_pos is None):
            target_pos = (0.0, 0.0, 0.35, 0.07, 1.02, -3.16, 1.27, 1.32, 0.0, 1.41, 'inf', 'inf',0.045,0.045)
        
        robot_parts={}
        for i, part_name in enumerate(RobotConst().PART_NAMES):
            robot_parts[part_name] = self.robot.getDevice(part_name)
            robot_parts[part_name].setPosition(float(target_pos[i]))
            robot_parts[part_name].setVelocity(robot_parts[part_name].getMaxVelocity() / 2.0)
        self.robot_parts = robot_parts
        return robot_parts
    
    def set_wheel_joint_vel(self, vL, vR):
        self.robot_parts["wheel_left_joint"].setVelocity(vL)
        self.robot_parts["wheel_right_joint"].setVelocity(vR)

    def manual_control(self, Mapping: MappingClass, vel_ratio=1, filter_tol=0.5):
        MAX_SPEED = self.rConst.MAX_SPEED
        key = self.keyboard.getKey()
        
        vL, vR = 0, 0
        while(self.keyboard.getKey() != -1): pass
        if key == self.keyboard.LEFT :
            vL, vR = -MAX_SPEED, MAX_SPEED
        elif key == self.keyboard.RIGHT:
            vL, vR = MAX_SPEED, -MAX_SPEED
        elif key == self.keyboard.UP:
            vL, vR = MAX_SPEED, MAX_SPEED
        elif key == self.keyboard.DOWN:
            vL, vR = -MAX_SPEED, -MAX_SPEED
        elif key == ord(' '):
            vL, vR = 0, 0
        elif key == ord('S'):
            Mapping.Map.save()
        elif key == ord('L'):
            map = Mapping.Map.load()
            Mapping.display_point_cloud(self.display, None, redraw=True)
        elif key == ord('D'):
            Mapping.Map.display()
        elif key == ord('F'):
            map = Mapping.Map.filter(tol=filter_tol)
            Mapping.Map.display(map)
        return vL*vel_ratio, vR*vel_ratio

    def enable_gripper_encoders(self):
        # Enable gripper encoders (position sensors)
        left_gripper_enc  = self.robot.getDevice("gripper_left_finger_joint_sensor")
        right_gripper_enc = self.robot.getDevice("gripper_right_finger_joint_sensor")
        left_gripper_enc.enable(self.timestep)
        right_gripper_enc.enable(self.timestep)
        return left_gripper_enc, right_gripper_enc

    def enable_camera(self):
        # Enable Camera
        camera = self.robot.getDevice('camera')
        camera.enable(self.timestep)
        camera.recognitionEnable(self.timestep)
        return camera

    def enable_gps_compass(self):
        # Enable GPS and compass localization
        gps = self.robot.getDevice("gps")
        gps.enable(self.timestep)

        compass = self.robot.getDevice("compass")
        compass.enable(self.timestep)
        return gps, compass

    def enable_lidar(self):
        # Enable LiDAR
        lidar = self.robot.getDevice('Hokuyo URG-04LX-UG01')
        lidar.enable(self.timestep)
        lidar.enablePointCloud()
        return lidar

    def enable_display(self):
        # Enable display
        display = self.robot.getDevice("display")
        return display
    
    def enable_keyboard(self):
        # We are using a keyboard to remote control the robot
        keyboard = self.robot.getKeyboard()
        keyboard.enable(self.timestep)
        return keyboard