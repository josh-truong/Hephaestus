import py_trees
import cv2
import numpy as np
import matplotlib.pyplot as plt


class LineDetection(py_trees.behaviour.Behaviour):
    def __init__(self, name, writer, reader):
        super(LineDetection, self).__init__(name)
        self.logger.debug("%s [%s::__init__()]" % (self.name, self.__class__.__name__))
        self.w, self.r = writer, reader

    def log_message(self, function_name: str, feedback_message=""):
        self.logger.debug("%s [%s::%s][%s]" % (self.name, function_name, self.__class__.__name__, feedback_message))

    def setup(self):
        self.log_message("setup()")
        self.low_tol, self.high_tol = 100, 150
        self.rho = 1  # distance resolution in pixels of the Hough grid
        self.theta = np.pi / 180  # angular resolution in radians of the Hough grid
        self.threshold = 30  # minimum number of votes (intersections in Hough grid cell)
        self.min_line_length = 50  # minimum number of pixels making up a line
        self.max_line_gap = 30  # maximum gap in pixels between connectable line segments
        self.counter = 0
        self.frequency = self.r.env.refresh_hz*3

    def initialise(self):
        self.log_message("initialise()")

    def update(self):
        self.counter += 1
        self.log_message("update()", f"Line Detecting in {self.frequency - self.counter}")
        if (self.counter%self.frequency == 0):
            self.counter = 0

            img = self.r.env.map.map
            img = np.uint8(img*255) # Add a filter method here
            edges = cv2.Canny(img, self.low_tol, self.high_tol)
            line_image = np.copy(img) * 0  # creating a blank to draw lines on
            # Run Hough on edge detected image
            lines = cv2.HoughLinesP(edges, self.rho, self.theta, self.threshold, np.array([]),
                                self.min_line_length, self.max_line_gap)
            if (lines is None): return py_trees.common.Status.SUCCESS
            lines = np.row_stack(lines).reshape((-1, 1, 2))
            # for x1,y1,x2,y2 in lines:
            #     cv2.line(line_image,(x1,y1),(x2,y2),(255,0,0),5)
            line_image = cv2.polylines(line_image,lines,False,(255,0,0),5)
            # self.r.env.map.map = np.clip(img - np.mean(img - np.clip(line_image, 0, 1)), 0, 1)
        return py_trees.common.Status.SUCCESS
        
    def terminate(self, new_status):
        self.log_message("terminate()", "%s->%s" % (self.status, new_status))
