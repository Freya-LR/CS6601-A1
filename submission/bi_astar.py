#####################################################
# CS 6601 - Assignment 1͏︌͏󠄁͏︉
# bi_astar.py͏︌͏󠄁͏︉
#####################################################

# DO NOT ADD OR REMOVE ANY IMPORTS FROM THIS FILE͏︌͏󠄁͏︉
import math
from submission.priority_queue import PriorityQueue
from submission.astar import euclidean_dist_heuristic

# Credits if any͏︌͏󠄁͏︉
# 1)͏︌͏󠄁͏︉
# 2)͏︌͏󠄁͏︉
# 3)͏︌͏󠄁͏︉

def bidirectional_a_star(graph, start, goal,
                         heuristic=euclidean_dist_heuristic) -> list:
    """
    Exercise 2: Bidirectional A*.

    See README.md for exercise description.

    Args:
        graph (ExplorableGraph): Undirected graph to search.
        start: Key for the start node.
        goal: Key for the end node.
        heuristic: Function to determine distance heuristic.
            Default: euclidean_dist_heuristic.

    Returns:
        The best path via bi-A* as a list from the start to the goal node (including both).
    """

    # TODO: finish this function!͏︌͏󠄁͏︉
    raise NotImplementedError