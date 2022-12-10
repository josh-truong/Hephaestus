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
        self.display = DisplayOverlays(self.w, self.r)
        self.get_display_coords = self.display.get_display_coords


    def initialise(self):
        self.log_message("initialise()")

    def update(self):
        self.log_message("update()", "KMeans is running...")
        object_location = np.array(self.r.env.object_location)
        raw_x, raw_y = object_location[:,0], object_location[:,1]

        n_cluster = self.r.env.num_objects
        rand_c = object_location
        np.random.shuffle(rand_c)
        cx, cy = rand_c[:n_cluster, 0], rand_c[:n_cluster, 1]

        kmeans = KmeansClustering()
        # objects moving simulation
        cx, cy = kmeans.update_positions(cx, cy)
        clusters = kmeans.run(raw_x, raw_y, n_cluster)
        centroids = np.vstack([clusters.center_x, clusters.center_y, np.zeros(clusters.n_label)]).T
        self.w.env.kmeans_location = centroids.tolist()
        self.editWaypoints()
        self.display.draw_estimated_location_on_map(self.r.env.kmeans_location, color=0x00FF00)
        return py_trees.common.Status.SUCCESS
        
    def terminate(self, new_status):
        self.log_message("terminate()", "%s->%s" % (self.status, new_status))



    def editWaypoints(self):
        map = self.r.env.map.map
        for i in range(len(self.r.env.kmeans_location)):
            wX,wY,depth = self.r.env.kmeans_location[i]
            x, y = self.display.get_display_coords(wX,wY)
            upOffset, downOffset = 0, 0
            while y + upOffset <= 360 and map[y + upOffset][x] == 1:
                upOffset += 1
            while y - downOffset >= 0 and map[y - downOffset,x] == 1:
                downOffset += 1

            upWorld = self.display.get_world_coords(x,y + upOffset)
            downWorld = self.display.get_world_coords(x,y - upOffset)

            if (y - downOffset == -1):
                self.w.env.kmeans_location[i] = [wX, upWorld[1], depth]
            elif (y + upOffset == 361):
                self.w.env.kmeans_location[i] = [wX, downWorld[1], depth]
            elif (upOffset < downOffset):
                self.w.env.kmeans_location[i] = [wX, upWorld[1], depth]
            else:
                self.w.env.kmeans_location[i] = [wX, downWorld[1], depth]