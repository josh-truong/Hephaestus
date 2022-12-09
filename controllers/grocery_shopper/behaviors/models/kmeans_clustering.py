"""
kmeans_clustering.py

Last updated on Fri Dec 9 2022
@Lead: Joshua Truong
- Modified into classes else is attributed to Atsushi Sakai
"""


"""
Object clustering with k-means algorithm
author: Atsushi Sakai (@Atsushi_twi)
"""
import math
import matplotlib.pyplot as plt
import random
import numpy as np

# k means parameters
MAX_LOOP = 10
DCOST_TH = 0.1

class Clusters:

    def __init__(self, x, y, n_label):
        self.x = x
        self.y = y
        self.n_data = len(self.x)
        self.n_label = n_label
        self.labels = [random.randint(0, n_label - 1)
                       for _ in range(self.n_data)]
        self.center_x = [0.0 for _ in range(n_label)]
        self.center_y = [0.0 for _ in range(n_label)]

    def plot_cluster(self):
        for label in set(self.labels):
            x, y = self._get_labeled_x_y(label)
            plt.plot(x, y, ".")

    def calc_centroid(self):
        for label in set(self.labels):
            x, y = self._get_labeled_x_y(label)
            n_data = len(x)
            self.center_x[label] = sum(x) / n_data
            self.center_y[label] = sum(y) / n_data

    def update_clusters(self):
        cost = 0.0

        for ip in range(self.n_data):
            px = self.x[ip]
            py = self.y[ip]

            dx = [icx - px for icx in self.center_x]
            dy = [icy - py for icy in self.center_y]

            dist_list = [math.hypot(idx, idy) for (idx, idy) in zip(dx, dy)]
            min_dist = min(dist_list)
            min_id = dist_list.index(min_dist)
            self.labels[ip] = min_id
            cost += min_dist

        return cost

    def _get_labeled_x_y(self, target_label):
        x = [self.x[i] for i, label in enumerate(self.labels) if label == target_label]
        y = [self.y[i] for i, label in enumerate(self.labels) if label == target_label]
        return x, y

class KmeansClustering:
    def run(self, rx, ry, nc):
        clusters = Clusters(rx, ry, nc)
        clusters.calc_centroid()

        pre_cost = float("inf")
        for loop in range(MAX_LOOP):
            print("loop:", loop)
            cost = clusters.update_clusters()
            clusters.calc_centroid()

            d_cost = abs(cost - pre_cost)
            if d_cost < DCOST_TH:
                break
            pre_cost = cost
        return clusters

    def update_positions(self, cx, cy):
        # object moving parameters
        DX1 = 0.4
        DY1 = 0.5
        DX2 = -0.3
        DY2 = -0.5

        cx[0] += DX1
        cy[0] += DY1
        cx[1] += DX2
        cy[1] += DY2

        return cx, cy