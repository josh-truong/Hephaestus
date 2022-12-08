"""
planning.py

Created on Tues Nov 29 2022
@Lead: Nathan Kochera
"""

import numpy as np
import matplotlib.pyplot as plt
import random
from scipy import interpolate


class Node:
    """
    Node for RRT Algorithm. This is what you'll make your graph with!
    """
    def __init__(self, pt, parent=None):
        self.point = pt # n-Dimensional point
        self.parent = parent # Parent node
        self.path_from_parent = [] # List of points along the way from the parent node (for edge's collision checking)

class Planning:
    def __init__(self):
        print("=== RRT Component Initialized...")

    def visualize_2D_graph(self, map, nodes, goal_point=None, filename=None):
        '''
        Function to visualise the 2D world, the RRT graph, path to goal if goal exists
        :param state_bounds: Array of min/max for each dimension
        :param obstacles: Locations and radii of spheroid obstacles
        :param nodes: List of vertex locations
        :param goal_point: Point within state_bounds to target with the RRT. (OPTIONAL, can be None)
        :param filename: Complete path to the file on which this plot will be saved
        :return: None
        '''
        fig = plt.figure()
        plt.xlim(0, len(map))
        plt.ylim(0, len(map[1]))

        goal_node = None
        for node in nodes:
            if node.parent is not None:
                node_path = np.array(node.path_from_parent)
                plt.plot(node_path[:,0], node_path[:,1], '-b')
            # The goal may not be on the RRT so we are finding the point that is a 'proxy' for the goal
            if goal_point is not None and np.linalg.norm(node.point - np.array(goal_point)) <= 1e-5:
                goal_node = node
                plt.plot(node.point[0], node.point[1], 'k^')
            else:
                plt.plot(node.point[0], node.point[1], 'ro')

        plt.plot(nodes[0].point[0], nodes[0].point[1], 'ko')

        if goal_node is not None:
            cur_node = goal_node
            while cur_node is not None: 
                if cur_node.parent is not None:
                    node_path = np.array(cur_node.path_from_parent)
                    plt.plot(node_path[:,0], node_path[:,1], '-y')
                    cur_node = cur_node.parent
                else:
                    break

        if goal_point is not None:
            plt.plot(goal_point[0], goal_point[1], 'gx')

        if filename is not None:
            fig.savefig(filename)
        else:
            plt.show()

        plt.imshow(map)
        plt.colorbar()
        plt.show()

    def get_nearest_vertex(self, node_list, q_point):
        '''
        Function that finds a node in node_list with closest node.point to query q_point
        :param node_list: List of Node objects
        :param q_point: n-dimensional array representing a point
        :return Node in node_list with closest node.point to query q_point
        '''
        # TODO: Your Code Here
        minDist = np.linalg.norm(node_list[0].point - q_point) # stores the minimum distance between q-point and minNode.point 
        minNode = node_list[0] # the closest node to q_point
        for i in range(1,len(node_list)):
            dist = np.linalg.norm(node_list[i].point - q_point)
            if dist < minDist:
                minDist = dist
                minNode = node_list[i]
        # loop finds the closest node to q_point
        return minNode

    def state_is_valid(self, point, map):
        if (map[int(point[1])][int(point[0])] == 0):
            return True
        return False
        
            
    def steer(self, from_point, to_point, delta_q):
        '''
        :param from_point: n-Dimensional array (point) where the path to "to_point" is originating from (e.g., [1.,2.])
        :param to_point: n-Dimensional array (point) indicating destination (e.g., [0., 0.])
        :param delta_q: Max path-length to cover, possibly resulting in changes to "to_point" (e.g., 0.2)
        :return path: Array of points leading from "from_point" to "to_point" (inclusive of endpoints)  (e.g., [ [1.,2.], [1., 1.], [0., 0.] ])
        '''
        # TODO: Figure out if you can use "to_point" as-is, or if you need to move it so that it's only delta_q distance away
        if np.linalg.norm(from_point - to_point) > delta_q: # gets the distance between the two points and sees if it is greater than delta_q
            # if greater than delta_q normalize vector pointing from from_point to to_point, make it the length of delta_q, 
            # and add it to from_point to get the new to_point
            vector = to_point - from_point
            unit_vector = vector / np.linalg.norm(vector)
            change_vector = unit_vector * (delta_q - (delta_q / 100000)) # subtracting small amount from delta_q because sometimes the unit vector is normalized to something slightly bigger than 1
            to_point = from_point + change_vector
            to_point = np.floor(to_point)
        # TODO Use the np.linspace function to get 10 points along the path from "from_point" to "to_point"
        path = np.floor(np.linspace(from_point, to_point, 10))
        return path

    def check_path_valid(self, path, map):
        '''
        Function that checks if a path (or edge that is made up of waypoints) is collision free or not
        :param path: A 1D array containing a few (10 in our case) n-dimensional points along an edge
        :param state_is_valid: Function that takes an n-dimensional point and checks if it is valid
        :return: Boolean based on whether the path is collision free or not
        '''
        # TODO: Your Code Here
        valid = True
        for point in path:
            valid = valid and self.state_is_valid(point, map) # checks each point in the path, valid will become false if any of them are not valid points
        return valid

    def rrt(self, starting_point, goal_point, k, delta_q, map):
        '''
        TODO: Implement the RRT algorithm here, making use of the provided state_is_valid function.
        RRT algorithm.
        If goal_point is set, your implementation should return once a path to the goal has been found 
        (e.g., if q_new.point is within 1e-5 distance of goal_point), using k as an upper-bound for iterations. 
        If goal_point is None, it should build a graph without a goal and terminate after k iterations.

        :param state_bounds: matrix of min/max values for each dimension (e.g., [[0,1],[0,1]] for a 2D 1m by 1m square)
        :param state_is_valid: function that maps states (N-dimensional Real vectors) to a Boolean (indicating free vs. forbidden space)
        :param starting_point: Point within state_bounds to grow the RRT from
        :param goal_point: Point within state_bounds to target with the RRT. (OPTIONAL, can be None)
        :param k: Number of points to sample
        :param delta_q: Maximum distance allowed between vertices
        :returns List of RRT graph nodes
        '''
        plt.imshow(map)
        plt.pause(0.01)
        node_list = []
        node_list.append(Node(starting_point, parent=None)) # Add Node at starting point with no parent
        # TODO: Your code here
        # TODO: Make sure to add every node you create onto node_list, and to set node.parent and node.path_from_parent for each
        for i in range(k):
            # create a random new point, setting it to the goal_point a small portion of the time
            rand = np.array([])
            if goal_point is not None and random.random() < 0.05:
                rand = goal_point
            else:
                rand = np.array([np.random.randint(0, len(map)), np.random.randint(0, len(map[1]))])
            # get closest node to new point
            near = self.get_nearest_vertex(node_list, rand)
            # get a path to the new point, moving it closer to the closest node if necessary
            pathToNew = self.steer(near.point, rand, delta_q)
            # check if the path is valid, and only then add new node to the node_list
            if self.check_path_valid(pathToNew, map):
                newNode = Node(pathToNew[9], parent=near)
                newNode.path_from_parent = pathToNew
                node_list.append(newNode)
                # if there is a goal, check if the goal has been found and return
                if goal_point is not None:
                    if np.linalg.norm(newNode.point - goal_point) < 1e-5:
                        return node_list
        print("Path not found.")
        return node_list


    def get_world_coords(self, x, y, display=(360, 360), world=(30, 15)):
        x = ((display[0]*0.5) - x) / (display[0]/world[0])
        y = (y + (display[1]*0.5) - display[1]) / (display[1]/world[1])
        return [x,-y]

    def getWaypoints(self, nodes, display_coords=False):
        # list of waypoints in map coords, tulpes with (x, y, theta)
        waypoints = []
        goal_node = nodes[-1]
        if goal_node is not None:
            cur_node = goal_node
            while cur_node is not None: 
                if cur_node.parent is not None:
                    waypoints.extend(cur_node.path_from_parent)
                    cur_node = cur_node.parent
                else:
                    waypoints.reverse()
                    # print(waypoints[0], waypoints[-1])
                    waypoints = [self.get_world_coords(x, y) for x,y in waypoints]
                    # try:
                    #     waypoints = self.smooth_path(waypoints)[:-1]
                    #     return waypoints
                    # except:
                    #     return waypoints
                    return waypoints

    def smooth_path(self, waypoints):
        if (waypoints is None): return []
        waypoints = np.array(waypoints)
        x, y = waypoints[:,0], waypoints[:,1]

        # Remove duplicate points
        okay = np.where(np.abs(np.diff(x)) + np.abs(np.diff(y)) > 0)[0]
        xn = np.r_[x[okay], x[-1], x[0]]
        yn = np.r_[y[okay], y[-1], y[0]]

        # create spline function
        tck, u = interpolate.splprep([xn, yn], s=0, k=3, per=True)

        #create interpolated lists of points
        xn, yn = interpolate.splev(np.linspace(0, 1, 20), tck)
        return list(zip(xn, yn))


# if __name__ == "__main__":
#     planner = Planning()
#     K = 1000 # Feel free to adjust as desired
#     map = np.load("../../assets/filter_map.npy")

#     starting_point = [20,200]
#     goal = np.array([325, 325])
#     nodes = planner.rrt(starting_point, goal, K, 10, map)
#     planner.getWaypoints(nodes)
#     # planner.visualize_2D_graph(map, nodes, goal, 'rrt_run2.png')