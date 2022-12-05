import py_trees
from .models import MappingModel

class Mapping(py_trees.behaviour.Behaviour):
    def __init__(self, name, writer, reader):
        super(Mapping, self).__init__(name)
        self.logger.debug("%s [%s::__init__()]" % (self.name, self.__class__.__name__))
        self.w, self.r = writer, reader

    def log_message(self, function_name: str, feedback_message=""):
        self.logger.debug("%s [%s::%s][%s]" % (self.name, function_name, self.__class__.__name__, feedback_message))

    def setup(self):
        self.log_message("setup()")
        self.MapModel = MappingModel(self.w, self.r)

    def initialise(self):
        # self.log_message("initialise()")
        pass

    def update(self):
        pose = self.r.robot.pose
        point_cloud = self.MapModel.get_lidar_point_cloud(pose)
        self.MapModel.display_point_cloud(point_cloud)
        self.feedback_message = f"{len(point_cloud)} lidar points detected."
        self.log_message("update()", self.feedback_message)
        return py_trees.common.Status.SUCCESS
        
    def terminate(self, new_status):
        # self.log_message("terminate()", "%s->%s" % (self.status, new_status))
        pass