import py_trees
import numpy as np
from scipy.signal import convolve2d

class FilteringMap(py_trees.behaviour.Behaviour):
    def __init__(self, name, writer, reader):
        super(FilteringMap, self).__init__(name)
        self.logger.debug("%s [%s::__init__()]" % (self.name, self.__class__.__name__))
        self.w, self.r = writer, reader

    def log_message(self, function_name: str, feedback_message=""):
        self.logger.debug("%s [%s::%s][%s]" % (self.name, function_name, self.__class__.__name__, feedback_message))

    def setup(self):
        self.log_message("setup()")
        self.horizontal_mask = np.array([
            [-1, -1, -1],
            [ 2,  2,  2],
            [-1, -1, -1],
        ])
        self.vertical_mask = np.array([
            [-1, 2, -1],
            [-1, 2, -1],
            [-1, 2, -1],
        ])
        self.counter = 0
        self.frequency = self.r.env.refresh_hz

    def initialise(self):
        # self.log_message("initialise()")
        pass

    def update(self):
        def redraw_display(map):
            self.w.env.map.update_map(map)
            width, height = map.shape
            map = (map.T*255).astype(int)
            map = np.dstack([map, map, map])

            display = self.r.device.display
            ir = display.imageNew(map.tolist(), display.RGB, width, height)
            display.imagePaste(ir, 0, 0, False)
            display.imageDelete(ir)

        self.counter += 1
        self.feedback_message = f"Convolving Map in {self.frequency - self.counter}"
        if (self.counter%self.frequency == 0):
            self.feedback_message = "Convolving Map"
            map = self.r.env.map.map
            self.counter = 0

            grad = convolve2d(map, self.horizontal_mask, mode='same')
            grad += convolve2d(map, self.vertical_mask, mode='same')
            idx = grad >= self.r.env.ftol
            grad[~idx] = 0
            grad[idx] = map[idx]
            grad[idx] = np.clip(grad[idx], 0.0, 1.0)
            redraw_display(grad)
        self.log_message("update()", self.feedback_message)
        return py_trees.common.Status.SUCCESS
        
    def terminate(self, new_status):
        # self.log_message("terminate()", "%s->%s" % (self.status, new_status))
        pass
