import py_trees
from .models import SpeedController


class MoveForward(py_trees.behaviour.Behaviour):
    def __init__(self, name, writer, reader):
        super(MoveForward, self).__init__(name)
        self.logger.debug("%s [%s::__init__()]" % (self.name, self.__class__.__name__))
        self.w, self.r = writer, reader

    def log_message(self, function_name: str, feedback_message=""):
        self.logger.debug("%s [%s::%s][%s]" % (self.name, function_name, self.__class__.__name__, feedback_message))

    def setup(self):
        # self.log_message("setup()")
        self.controller = SpeedController(self.w, self.r)

    def initialise(self):
        # self.log_message("initialise()")
        pass

    def update(self):
        self.feedback_message = ""
        self.log_message("update()", self.feedback_message)

        speed = self.r.constants.robot.MAX_SPEED
        vL, vR = speed*0.2, speed*0.2
        self.w.robot.vL, self.w.robot.vR = vL, vR
        self.controller.set_wheel_joint_vel(vL, vR)
        self.controller.localization.update_odometry()
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        # self.log_message("terminate()", "%s->%s" % (self.status, new_status))
        pass
