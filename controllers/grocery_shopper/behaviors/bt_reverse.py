import py_trees

class Reverse(py_trees.behaviour.Behaviour):
    def __init__(self, name, writer, reader):
        super(Reverse, self).__init__(name)
        self.logger.debug("%s [%s::__init__()]" % (self.name, self.__class__.__name__))
        self.w, self.r = writer, reader

    def log_message(self, function_name: str, feedback_message=""):
        self.logger.debug("%s [%s::%s][%s]" % (self.name, function_name, self.__class__.__name__, feedback_message))

    def setup(self):
        self.log_message("setup()")
        self.Driver = ControllerModel(self.w, self.r)

    def initialise(self):
        self.log_message("initialise()")
        self.counter = 0

    def update(self):
        if (not self.r.robot.reverse): return py_trees.common.Status.SUCCESS
        if (self.counter <= 5):
            robot = self.r.robot.robot
            if (robot.step(int(robot.getBasicTimeStep())) != -1):
                self.w.robot.vL, self.w.robot.vR = -2, -2
                self.Driver.set_wheel_joint_vel(vL, vR)
                self.Driver.localization.update_odometry()
            return py_trees.common.Status.RUNNING
        else:
            self.counter += 1
            self.w.robot.reverse = False
            self.w.env.rerun_rrt = True
            return py_trees.common.Status.SUCCESS


        self.feedback_message = ""
        self.log_message("update()", self.feedback_message)
        
        # 
        
    def terminate(self, new_status):
        self.log_message("terminate()", "%s->%s" % (self.status, new_status))
