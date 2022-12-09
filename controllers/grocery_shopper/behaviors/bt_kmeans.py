import py_trees
import numpy as np
import matplotlib.pyplot as plt

from .models import DisplayOverlays
from .models import KmeansClustering


class KMeans(py_trees.behaviour.Behaviour):
    def __init__(self, name, writer, reader):
        super(KMeans, self).__init__(name)
        self.logger.debug("%s [%s::__init__()]" % (self.name, self.__class__.__name__))
        self.w, self.r = writer, reader

    def log_message(self, function_name: str, feedback_message=""):
        self.logger.debug("%s [%s::%s][%s]" % (self.name, function_name, self.__class__.__name__, feedback_message))

    def setup(self):
        self.log_message("setup()")
        self.get_display_coords = DisplayOverlays(self.w, self.r).get_display_coords


    def initialise(self):
        self.log_message("initialise()")

    def update(self):

        object_location = self.r.env.object_location
        raw_x, raw_y = object_location[:,0], object_location[:,1]

        n_cluster = 15
        rand_c = object_location
        np.random.shuffle(rand_c)
        cx, cy = rand_c[:n_cluster, 0], rand_c[:n_cluster, 1]


        sim_time = 15.0
        dt = 1.0
        time = 0.0

        kmeans = KmeansClustering()
        while time <= sim_time:
            time += dt

            # objects moving simulation
            cx, cy = kmeans.update_positions(cx, cy)
            clusters = kmeans.run(raw_x, raw_y, n_cluster)


        # clusters.plot_cluster()
        plt.plot(cx, cy, "or")
        plt.show()

        self.feedback_message = ""
        self.log_message("update()", self.feedback_message)
        return py_trees.common.Status.SUCCESS
        # return py_trees.common.Status.RUNNING
        
    def terminate(self, new_status):
        self.log_message("terminate()", "%s->%s" % (self.status, new_status))
