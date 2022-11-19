from controller import Robot
from .constants import RobotConst

class Device:
    def __init__(self):
        print("=== Initializing Device Component...")
        # create the Robot instance.
        self.robot = Robot()

        # get the time step of the current world.
        self.timestep = int(self.robot.getBasicTimeStep())
    
    def robot_step(self):
        return self.robot.step(self.timestep)

    def get_timestep(self):
        # self.timestep = int(self.robot.getBasicTimeStep())
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
        return robot_parts

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