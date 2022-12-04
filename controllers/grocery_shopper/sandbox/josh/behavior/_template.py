import py_trees


class Template(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(ComputerVision, self).__init__(name)
        self.logger.debug("%s [%s::__init__()]" % (self.name, self.__class__.__name__))
        
    def log_message(self, function_name: str, feedback_message=""):
        self.logger.debug("%s [%s::%s][%s]" % (self.name, function_name, self.__class__.__name__, feedback_message))

    def setup(self):
        self.log_message("setup()")

    def initialise(self):
        self.log_message("initialise()")
        self.counter = 0

    def update(self):
        self.feedback_message = ""
        self.log_message("update()", self.feedback_message)
        return py_trees.common.Status.SUCCESS
        # return py_trees.common.Status.RUNNING
        
    def terminate(self, new_status):
        self.log_message("terminate()", "%s->%s" % (self.status, new_status))





py_trees.logging.level = py_trees.logging.Level.DEBUG

# class Nested(object):
#     def __init__(self):
#         self.foo = None
#         self.bar = None

#     def __str__(self):
#         return str(self.__dict__)

# writer = py_trees.blackboard.Client(name="Writer")
# writer.register_key(key="nested", access=py_trees.common.Access.WRITE)
# reader = py_trees.blackboard.Client(name="Reader")
# reader.register_key(key="nested", access=py_trees.common.Access.READ)

# writer.nested = Nested()
# writer.nested.foo = "I am foo"
# writer.nested.bar = "I am bar"

# foo = reader.nested.foo
# print(writer)
# print(reader)



root = py_trees.composites.Sequence("Sequence")
root.add_child(ComputerVision(name="Detecting"))
root.setup_with_descendants()
for i in range(2):
    root.tick_once()
