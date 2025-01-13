#####################################################
# CS 6601 - Assignment 1͏︌͏󠄁͏︉
# astar.py͏︌͏󠄁͏︉
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


def a_star(graph, start, goal, heuristic=euclidean_dist_heuristic) -> list:
    """
    Warm-up exercise: Implement A* algorithm.

    See README.md for exercise description.

    Args:
        graph (ExplorableGraph): Undirected graph to search.
        start: Key for the start node.
        goal: Key for the end node.
        heuristic: Function to determine distance heuristic.
            Default: euclidean_dist_heuristic.

    Returns:
        The best path via A* as a list from the start to the goal node (including both).
    """

    # TODO: finish this function!͏︌͏󠄁͏︉
    raise NotImplementedError