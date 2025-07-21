#####################################################
# CS 6601 - Assignment 1͏︌͏󠄁͏︉
# bfs.py͏︌͏󠄁͏︉
#####################################################

# DO NOT ADD OR REMOVE ANY IMPORTS FROM THIS FILE͏︌͏󠄁͏︉
import math

# Credits if any͏︌͏󠄁͏︉
# 1)͏︌͏󠄁͏︉
# 2)͏︌͏󠄁͏︉
# 3)͏︌͏󠄁͏︉

def return_your_name() -> str:
    """Return your first and last name from this function as a string"""

    return "Dongning Li"


def breadth_first_search(graph, start, goal) -> list:
    """
    Warm-up exercise: Implement breadth-first-search.

    See README.md for exercise description.

    Args:
        graph (ExplorableGraph): Undirected graph to search.
        start: Key for the start node.
        goal: Key for the end node.

    Returns:
        The best path via BFS as a list from the start to the goal node (including both).
    """
    # if the start and goal are the same, return empty list
    if start == goal:
        return []
    
    # initializa the frontier and reached node
    frontier = [[start]] # initialized frontier only contains start node
    reached = set([start]) # a set to track unique explored nodes
    
    while frontier:
        path = frontier.pop(0)
        node = path[-1]
        
        # track each neighbor of nodes
        for neighbor in sorted(graph.neighbors(node)):

            # if the neighbor is the goal, return current path with the neighbor
            if neighbor == goal:
                return path + [neighbor]
            
            # if the neighbor is not the goal and has not been explored
            if neighbor not in reached:
                reached.add(neighbor) # add the neighbor to reached set
                frontier.append(path+[neighbor]) # add new path to frontier queue

    # TODO: finish this function!͏︌͏󠄁͏︉
    # raise NotImplementedError
    # if the goal not found
    return "failure"