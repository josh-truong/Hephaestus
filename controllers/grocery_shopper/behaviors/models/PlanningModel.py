"""
PLanningModel.py

Last updated on Fri Dec 9 2022
@Lead: Nathan Kochera
"""


import numpy as np
import matplotlib.pyplot as plt
import random
from scipy import interpolate


class Node:
    """
    Node for RRT Algorithm
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
        :param map: the map of the environment
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
            # The goal may not be on the RRT so we are finding the point that is a stand-in for the goal
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
        :param q_point: 2-dimensional array representing a point
        :return Node in node_list with closest node.point to query q_point
        '''
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
        :param from_point: 2-Dimensional array (point) where the path to "to_point" is originating from 
        :param to_point: 2-Dimensional array (point) indicating destination 
        :param delta_q: Max path-length to cover, possibly resulting in changes to "to_point" 
        :return path: Array of points leading from "from_point" to "to_point" (inclusive of endpoints)
        '''
        if np.linalg.norm(from_point - to_point) > delta_q: # gets the distance between the two points and sees if it is greater than delta_q
            # if greater than delta_q normalize vector pointing from from_point to to_point, make it the length of delta_q, 
            # and add it to from_point to get the new to_point
            vector = to_point - from_point
            unit_vector = vector / np.linalg.norm(vector)
            change_vector = unit_vector * (delta_q - (delta_q / 100000)) # subtracting small amount from delta_q because sometimes the unit vector is normalized to something slightly bigger than 1
            to_point = from_point + change_vector
            to_point = np.floor(to_point)
        path = np.floor(np.linspace(from_point, to_point, 10))
        return path

    def check_path_valid(self, path, map):
        '''
        Function that checks if a path is colliding with any obstacles
        :param path: A 1D array containing 2-dimensional points
        :param map: the map to check against
        :return: Boolean based on whether the path is collision free or not
        '''
        valid = True
        for point in path:
            valid = valid and self.state_is_valid(point, map) # checks each point in the path, valid will become false if any of them are not valid points
        return valid

    def rrt(self, starting_point, goal_point, k, delta_q, map):
        '''
        :param starting_point: Point within state_bounds to grow the RRT from
        :param goal_point: Point within state_bounds to target with the RRT. (OPTIONAL, can be None)
        :param k: Number of points to sample
        :param delta_q: Maximum distance allowed between vertices
        :returns List of RRT graph nodes
        '''
        
        node_list = []
        node_list.append(Node(starting_point, parent=None)) # Add Node at starting point with no parent
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
        print(f"RRT could not find path from {starting_point} to {goal_point}.")
        return node_list


    # get world coords from display coords
    def get_world_coords(self, x, y, display=(360, 360), world=(30, 15)):
        x = ((display[0]*0.5) - x) / (display[0]/world[0])
        y = (y + (display[1]*0.5) - display[1]) / (display[1]/world[1])
        return [x,-y]

    def getWaypoints(self, nodes, display_coords=False, smooth=True):
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
                    try:
                        if (smooth):
                            waypoints = self.smooth_path(waypoints)[:-1]
                            new_waypoints = []
                            for i in range(len(waypoints)-1):
                                start, end = waypoints[i], waypoints[i+1]
                                num_pixels_between = abs(start[0]-end[0])+abs(start[1]-end[1])
                                new_waypoints.extend(np.linspace(start, end, int(num_pixels_between)))
                            waypoints = [self.get_world_coords(x, y) for x,y in new_waypoints]
                            return waypoints
                        else:
                            waypoints = [self.get_world_coords(x, y) for x,y in waypoints]
                            return waypoints
                    except:
                        waypoints = [self.get_world_coords(x, y) for x,y in waypoints]
                        return waypoints
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
