#####################################################
# CS 6601 - Assignment 1͏︌͏󠄁͏︉
# race.py͏︌͏󠄁͏︉
#####################################################

# DO NOT ADD OR REMOVE ANY IMPORTS FROM THIS FILE͏︌͏󠄁͏︉
import math
from submission.priority_queue import PriorityQueue

# Credits if any͏︌͏󠄁͏︉
# 1)͏︌͏󠄁͏︉
# 2)͏︌͏󠄁͏︉
# 3)͏︌͏󠄁͏︉

def null_heuristic(graph, u, v):
    """
    Null heuristic used as a base line.

    Args:
        graph (ExplorableGraph): Undirected graph to search.
        u: Key for the first node to calculate from.
        v: Key for the second node to calculate to.

    Returns:
        0
    """

    return 0


def euclidean_dist_heuristic(graph, u, v):
    """
    Warm-up exercise: Implement the euclidean distance heuristic.

    See README.md for exercise description.

    Args:
        graph (ExplorableGraph): Undirected graph to search.
        u: Key for the first node to calculate from.
        v: Key for the second node to calculate to.

    Returns:
        Euclidean distance between the u node and the v node
        Round the result to 3 decimal places (if applicable)
    """

    # TODO: finish this function!͏︌͏󠄁͏︉
    raise NotImplementedError


def haversine_dist_heuristic(graph, u, v):
    """
    Note: This provided heuristic is for the Atlanta race.

    Args:
        graph (ExplorableGraph): Undirected graph to search.
        u: Key for the node to calculate from.
        v: Key for the end node to calculate to.

    Returns:
        Haversine distance between u node and v node
    """

    #Load latitude and longitude coordinates in radians:͏︌͏󠄁͏︉
    uLatLong = (math.radians(graph.nodes[u]["pos"][0]), math.radians(graph.nodes[u]["pos"][1]))
    vLatLong = (math.radians(graph.nodes[v]["pos"][0]), math.radians(graph.nodes[v]["pos"][1]))

    #Now we want to execute portions of the formula:͏︌͏󠄁͏︉
    constOutFront = 2*6371 #Radius of Earth is 6,371 kilometers
    term1InSqrt = (math.sin((vLatLong[0]-uLatLong[0])/2))**2 #First term inside sqrt
    term2InSqrt = math.cos(uLatLong[0])*math.cos(vLatLong[0])*((math.sin((vLatLong[1]-uLatLong[1])/2))**2) #Second term
    return constOutFront*math.asin(math.sqrt(term1InSqrt+term2InSqrt)) #Straight application of formula


def compute_landmarks(graph) -> list:
    """
    (Optional)
    Feel free to implement this method for computing landmarks. You can call
    compute_landmarks() from within load_data().

    Args:
        graph (ExplorableGraph): Undirected graph to search.

    Returns:
        List with not more than 4 computed landmarks. 
    """

    # TODO: finish this function͏︌͏󠄁͏︉
    return None


def load_data(graph, time_left):
    """
    Feel free to implement this method. We'll call it only once 
    at the beginning of the Race, and we'll pass the output to your custom_search function.
    graph: a networkx graph
    time_left: function you can call to keep track of your remaining time.
        usage: time_left() returns the time left in milliseconds.
        the max time will be 10 minutes.

    * To get a list of nodes, use graph.nodes()
    * To get node neighbors, use graph.neighbors(node)
    * To get edge weight, use graph.get_edge_weight(node1, node2)
    
    Returns:
        Any object that may be passed as the data argument to custom_search()
    """

    # nodes = graph.nodes()͏︌͏󠄁͏︉
    return None


def custom_search(graph, start, goal, data=None) -> list:
    """
    Race!: Implement your best search algorithm here to compete against the
    other student agents.

    If you implement this function and submit your code to Gradescope, you'll be
    registered for the Race!

    See README.md for exercise description.

    Args:
        graph (ExplorableGraph): Undirected graph to search.
        start: Key for the start node.
        goal: Key for the end node.
        data :  Data used in the custom search.
            Will be passed your data from load_data(graph).
            Default: None.

    Returns:
        The best path as a list from the start to the goal node (including both).
    """

    # TODO: finish this function!͏︌͏󠄁͏︉
    raise NotImplementedError