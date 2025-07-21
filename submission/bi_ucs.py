#####################################################
# CS 6601 - Assignment 1͏︌͏󠄁͏︉
# bi_ucs.py͏︌͏󠄁͏︉
#####################################################

# DO NOT ADD OR REMOVE ANY IMPORTS FROM THIS FILE͏︌͏󠄁͏︉
import math
from submission.priority_queue import PriorityQueue

# Credits if any͏︌͏󠄁͏︉
# 1)͏︌͏󠄁͏︉
# 2)͏︌͏󠄁͏︉
# 3)͏︌͏󠄁͏︉

def bidirectional_ucs(graph, start, goal) -> list:
    """
    Exercise 1: Bidirectional Search.

    See README.md for exercise description.

    Args:
        graph (ExplorableGraph): Undirected graph to search.
        start: Key for the start node.
        goal: Key for the end node.

    Returns:
        The best path via bi-UCS as a list from the start to the goal node (including both).
    """

    # TODO: finish this function!͏︌͏󠄁͏︉
    #raise NotImplementedError

    # If the start and goal are the same, return empty list
    if start == goal:
        return []
    
    # Initialize
    forward_frontier = PriorityQueue() # initialized forward frontier queue from PriorityQueue class
    forward_frontier.append([0,start]) # add start node, start cost is 0
    backward_frontier = PriorityQueue() # initialized backward frontier queue from PriorityQueue class
    backward_frontier.append([0,goal]) # add goal node, goal cost is 0
    forward_visited = {start:None} # node: its parent
    backward_visited = {goal:None} # node: its parent
    forward_cost = {start:0} # node:  cost so far to reach this node
    backward_cost = {goal:0} # node:  cost so far to reach this node

    meeting_node = None
    mu = float('inf')
    while forward_frontier and backward_frontier:
        # Expand forward frontier
        cost_f, node_f = forward_frontier.pop()
        # Expand backward frontier
        cost_b, node_b = backward_frontier.pop()
        
        # Stop when topf + topr ≥ μ.
        if cost_b+cost_f >=mu:
            break
        

        # Expand neighbors of forward frontier
        for neighbor in sorted(graph.neighbors(node_f)):

            # Cost so far for a neighbor equal to cost so far for current node 
            # plus current edge cost of current node and this neighbor
            neighbor_cost = forward_cost[node_f] + graph.get_edge_weight(node_f, neighbor) 
            
            # If this neighbor has not been in explored
            # or if it has been explored, but its cost so far of this path is smaller than explored cost so far
            if neighbor not in forward_cost or neighbor_cost < forward_cost[neighbor]:

                forward_cost[neighbor] = neighbor_cost # Add neighbor and cost so far into forward visited dict
                forward_frontier.append([neighbor_cost, neighbor]) # Add this neighbor and total cost to frontier queue
                forward_visited[neighbor] = node_f

        # Expand neighbors of backward frontier
        for neighbor in sorted(graph.neighbors(node_b)):

            # Cost so far for a neighbor equal to cost so far for current node 
            # plus current edge cost of current node and this neighbor
            neighbor_cost = backward_cost[node_b] + graph.get_edge_weight(node_b, neighbor) 
            
            # If this neighbor has not been in explored
            # or if it has been explored, but its cost so far of this path is smaller than explored cost so far
            if neighbor not in backward_cost or neighbor_cost < backward_cost[neighbor]:

                backward_cost[neighbor] = neighbor_cost # Add neighbor and cost so far into forward visited dict
                backward_frontier.append([neighbor_cost, neighbor]) # Add this neighbor and total cost to frontier queue
                backward_visited[neighbor] = node_b

        # Crossover node
        c_node = set(forward_cost.keys()).intersection(set(backward_cost.keys()))
        for node in c_node:
            if forward_cost[node]+backward_cost[node]<mu:
                mu = forward_cost[node]+backward_cost[node] # Update mu
                meeting_node = node

    # Reconstruct the path
    # If we get the meet node, add from start to goal
    path = []
    if meeting_node is not None:

        current = meeting_node
        # Forward path
        while current is not None:
            path.append(current) # Append node from the goal
            current = forward_visited[current] # Get the keys of each node
        path.reverse() # Reverse from start

        # Backforward path
        back_path=[]
        current = meeting_node
        while current is not None:
            back_path.append(current)
            current = backward_visited[current]
        path.extend(back_path[1:])
        
    return path
            



