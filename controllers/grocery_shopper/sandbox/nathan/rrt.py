from tabnanny import check
import numpy as np
import matplotlib.pyplot as plt
import math
import random

###############################################################################
## Base Code
###############################################################################
class Node:
    """
    Node for RRT Algorithm. This is what you'll make your graph with!
    """
    def __init__(self, pt, parent=None):
        self.point = pt # n-Dimensional point
        self.parent = parent # Parent node
        self.path_from_parent = [] # List of points along the way from the parent node (for edge's collision checking)


def setup_random_2d_world():
    '''
    Function that sets a 2D world with fixed bounds and # of obstacles
    :return: The bounds, the obstacles, the state_is_valid() function
    '''
    state_bounds = np.array([[0,10],[0,10]]) # matrix of min/max values for each dimension
    obstacles = [] # [pt, radius] circular obstacles
    for n in range(30):
        obstacles.append(get_nd_obstacle(state_bounds))

    def state_is_valid(state):
        '''
        Function that takes an n-dimensional point and checks if it is within the bounds and not inside the obstacle
        :param state: n-Dimensional point
        :return: Boolean whose value depends on whether the state/point is valid or not
        '''
        for dim in range(state_bounds.shape[0]):
            if state[dim] < state_bounds[dim][0]: return False
            if state[dim] >= state_bounds[dim][1]: return False
        for obs in obstacles:
            if np.linalg.norm(state - obs[0]) <= obs[1]: return False
        return True

    return state_bounds, obstacles, state_is_valid

def _plot_circle(x, y, radius, color="-k"):
    '''
    Internal function to plot a 2D circle on the current pyplot object
    :param x: The x coordinate of the circle
    :param y: The y coordinate of the circle
    :param radius: The radius of the circle
    :param color: Matplotlib color code
    :return: None
    '''
    deg = np.linspace(0,360,50)

    xl = [x + radius * math.cos(np.deg2rad(d)) for d in deg]
    yl = [y + radius * math.sin(np.deg2rad(d)) for d in deg]
    plt.plot(xl, yl, color)

def visualize_2D_graph(state_bounds, obstacles, nodes, goal_point=None, filename=None):
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
    plt.xlim(state_bounds[0,0], state_bounds[0,1])
    plt.ylim(state_bounds[1,0], state_bounds[1,1])

    for obs in obstacles:
        _plot_circle(obs[0][0], obs[0][1], obs[1])

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

def get_random_valid_vertex(state_is_valid, bounds):
    '''
    Function that samples a random n-dimensional point which is valid (i.e. collision free and within the bounds)
    :param state_valid: The state validity function that returns a boolean
    :param bounds: The world bounds to sample points from
    :return: n-Dimensional point/state
    '''
    vertex = None
    while vertex is None: # Get starting vertex
        pt = np.random.rand(bounds.shape[0]) * (bounds[:,1]-bounds[:,0]) + bounds[:,0]
        if state_is_valid(pt):
            vertex = pt
    return vertex

###############################################################################
## END BASE CODE
###############################################################################

def get_nearest_vertex(node_list, q_point):
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

def steer(from_point, to_point, delta_q):
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
    # TODO Use the np.linspace function to get 10 points along the path from "from_point" to "to_point"
    path = np.linspace(from_point, to_point, 10) 
    return path

def check_path_valid(path, state_is_valid):
    '''
    Function that checks if a path (or edge that is made up of waypoints) is collision free or not
    :param path: A 1D array containing a few (10 in our case) n-dimensional points along an edge
    :param state_is_valid: Function that takes an n-dimensional point and checks if it is valid
    :return: Boolean based on whether the path is collision free or not
    '''
    # TODO: Your Code Here
    valid = True
    for point in path:
        valid = valid and state_is_valid(point) # checks each point in the path, valid will become false if any of them are not valid points
    return valid

def rrt(state_bounds, state_is_valid, starting_point, goal_point, k, delta_q):
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
            rand = np.array([random.uniform(i[0], i[1]) for i in state_bounds])
        # get closest node to new point
        near = get_nearest_vertex(node_list, rand)
        # get a path to the new point, moving it closer to the closest node if necessary
        pathToNew = steer(near.point, rand, delta_q)
        # check if the path is valid, and only then add new node to the node_list
        if check_path_valid(pathToNew, state_is_valid):
            newNode = Node(pathToNew[9], parent=near)
            newNode.path_from_parent = pathToNew
            node_list.append(newNode)
            # if there is a goal, check if the goal has been found and return
            if goal_point is not None:
                if np.linalg.norm(newNode.point - goal_point) < 1e-5:
                    return node_list

    return node_list

if __name__ == "__main__":
    K = 250 # Feel free to adjust as desired

    bounds, obstacles, validity_check = setup_random_2d_world()
    starting_point = get_random_valid_vertex(validity_check, bounds)
    nodes = rrt(bounds, validity_check, starting_point, None, K, np.linalg.norm(bounds/10.))
    visualize_2D_graph(bounds, obstacles, nodes, None, 'rrt_run2.png')

    bounds, obstacles, validity_check = setup_random_2d_world()
    starting_point = get_random_valid_vertex(validity_check, bounds)
    goal_point = get_random_valid_vertex(validity_check, bounds)
    while np.linalg.norm(starting_point - goal_point) < np.linalg.norm(bounds/2.):
        starting_point = get_random_valid_vertex(validity_check, bounds)
        goal_point = get_random_valid_vertex(validity_check, bounds)
    nodes = rrt(bounds, validity_check, starting_point, goal_point, K, np.linalg.norm(bounds/10.))
    visualize_2D_graph(bounds, obstacles, nodes, goal_point, 'rrt_goal_run2.png')