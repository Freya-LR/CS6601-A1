#####################################################
# CS 6601 - Assignment 1͏︌͏󠄁͏︉
# ucs.py͏︌͏󠄁͏︉
#####################################################

# DO NOT ADD OR REMOVE ANY IMPORTS FROM THIS FILE͏︌͏󠄁͏︉
import math
from submission.priority_queue import PriorityQueue

# Credits if any͏︌͏󠄁͏︉
# 1)͏︌͏󠄁͏︉
# 2)͏︌͏󠄁͏︉
# 3)͏︌͏󠄁͏︉

def uniform_cost_search(graph, start, goal) -> list:
    """
    Warm-up exercise: Implement uniform_cost_search.

    See README.md for exercise description.

    Args:
        graph (ExplorableGraph): Undirected graph to search.
        start: Key for the start node.
        goal: Key for the end node.

    Returns:
        The best path via UCS as a list from the start to the goal node (including both).
    """

    # TODO: finish this function!͏︌͏󠄁͏︉
    # raise NotImplementedError

    # if the start and goal are the same, return empty list
    if start == goal:
        return []
    
    # Initialize
    frontier = PriorityQueue() # initialized frontier queue from PriorityQueue class
    frontier.append([0,start]) # add start node and cost 0 to frontier queue
    reached_cost = {start: 0} # store the minium cost to reach each node
    optional_path = {} # path as a dictionary to store previous node
    reached = set()

    while frontier:
        # Get the lowest cost node
        cost, node = frontier.pop()

        # If the node has explored, skip it
        if node in reached: 
            continue
        # Add this node to explored set
        reached.add(node)

        # If the goal is found, reconstruct the full path
        if node == goal:
            path = []
            while node != start:
                path.append(node) # Append node from the goal
                node = optional_path[node]# Get the keys of each node
            path.append(start) # Append start node
            path.reverse() # Reverse from start to goal
            return path

        # Track each neighbor of the node, sort alphabetically
        for neighbor in sorted(graph.neighbors(node)):

            # Cost so far for a neighbor equal to cost so far for current node 
            # plus current edge cost of current node and this neighbor
            neighbor_cost = cost + graph.get_edge_weight(node, neighbor) 
            
            # If this neighbor has not been in explored
            # or if it has been explored, but its cost so far of this path is smaller than explored cost so far
            if neighbor not in reached_cost or neighbor_cost < reached_cost[neighbor]:
                reached_cost[neighbor] = neighbor_cost # Add or replace cost so far 
                frontier.append([neighbor_cost, neighbor]) # Add this neighbor to frontier queue
                optional_path[neighbor] = node # Add the neighbor as the key to current node 

    #return optional_path
    return []