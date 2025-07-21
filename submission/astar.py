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
    # raise NotImplementedError

    # Get the position of two nodes u and v in graph
    u_pos = graph.nodes[u]['pos']
    v_pos = graph.nodes[v]['pos']
    # Euclidean distance = square root of ((u[x]-v[x])**2+(u[y]-v[y])**2)
    dist = math.sqrt(sum((u_pos[i]-v_pos[i])**2 for i in range(len(u_pos))))
    # Return the distance and Round the result to 3 decimal places
    return round(dist, 3)


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
    # raise NotImplementedError

    # if the start and goal are the same, return empty list
    if start == goal:
        return []
    
    # Initialize
    frontier = PriorityQueue() # initialized frontier queue from PriorityQueue class
    frontier.append([heuristic(graph, start, goal),start]) # add start node, start cost is euclidean dist
    reached_cost = {start: 0} # cost so far as a dictionary
    node_dict = {} # path as a dictionary
    reached_node = set() # Initialize a set to store explored nodes

    while frontier:
        # Get the top priority node of frontiers which is the least cost node
        cost, node = frontier.pop()

        # If the node has explored, skip it
        if node in reached_node: 
            continue
        # Add this node to explored set
        reached_node.add(node)

        # If the goal is found, reconstruct the full path
        if node == goal:
            path = []
            while node != start:
                path.append(node) # Append node from the goal
                node = node_dict[node] # Get the keys of each node
            path.append(start) # Append start node
            path.reverse() # Reverse from start
            return path

        # Track each neighbor of the node, sort alphabetically
        for neighbor in sorted(graph.neighbors(node)):

            # Cost so far for a neighbor equal to cost so far for current node 
            # plus current edge cost of current node and this neighbor
            neighbor_cost = reached_cost[node] + graph.get_edge_weight(node, neighbor) 
            
            # If this neighbor has not been in explored
            # or if it has been explored, but its cost so far of this path is smaller than explored cost so far
            if neighbor not in reached_cost or neighbor_cost < reached_cost[neighbor]:

                reached_cost[neighbor] = neighbor_cost # Add or replace cost so far 
                total_cost = neighbor_cost + heuristic(graph, neighbor, goal) # Calculate estimated total cost
                frontier.append([total_cost, neighbor]) # Add this neighbor to frontier queue
                node_dict[neighbor] = node # Set the neighbor as the key of current node

    #return reached_cost, optional_path

    return []
            
            

