#####################################################
# CS 6601 - Assignment 1͏︌͏󠄁͏︉
# tri_astar.py͏︌͏󠄁͏︉
#####################################################

# DO NOT ADD OR REMOVE ANY IMPORTS FROM THIS FILE͏︌͏󠄁͏︉
import math
from submission.priority_queue import PriorityQueue
from submission.astar import euclidean_dist_heuristic

# Credits if any͏︌͏󠄁͏︉
# 1)͏︌͏󠄁͏︉
# 2)͏︌͏󠄁͏︉
# 3)͏︌͏󠄁͏︉

def custom_heuristic(graph, u, v):
    """
        Feel free to use this method to try and work with different heuristics and come up with a better search algorithm.
        Args:
            graph (ExplorableGraph): Undirected graph to search.
            u (str): Key for the first node to calculate from.
            v (str): Key for the second node to calculate to.
        Returns:
            Custom heuristic distance between `u` node and `v` node
        """
    pass


def tridirectional_upgraded(graph, goals, heuristic=euclidean_dist_heuristic) -> list:
    """
    Exercise 4: Upgraded Tridirectional Search

    See README.MD for exercise description.

    Args:
        graph (ExplorableGraph): Undirected graph to search.
        goals (list): Key values for the 3 goals
        heuristic: Function to determine distance heuristic.
            Default: euclidean_dist_heuristic.

    Returns:
        The best path as a list from one of the goal nodes (including both of
        the other goal nodes).
    """

    # TODO: finish this function͏︌͏󠄁͏︉
    raise NotImplementedError