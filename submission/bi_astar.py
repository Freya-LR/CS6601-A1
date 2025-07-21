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

    if start == goal:
        return []
    
    # Initialize
    forward_frontier = PriorityQueue() # initialized frontier queue from PriorityQueue class
    forward_frontier.append([heuristic(graph, start, goal),start]) # add start node, start cost is euclidean dist to goal
    backward_frontier = PriorityQueue()
    backward_frontier.append([heuristic(graph, start, goal),goal])
    forward_visited = {start:None} # node: its parent
    backward_visited = {goal:None} # node: its parent
    forward_cost = {start:0} # node:  cost so far to reach this node
    backward_cost = {goal:0} # node:  cost so far to reach this node
    f_node = set() # forward explored set
    b_node = set() # backward explored set
    meeting_node = None
    mu = float('inf')

    while forward_frontier and backward_frontier:
        
        # Expand forward frontier
        cost_f, node_f = forward_frontier.pop()     

        # Expand backward frontier
        cost_b, node_b = backward_frontier.pop()

        if not forward_frontier or not backward_frontier:
            break

        # Stop when topf + topr ≥ μ.
        if cost_b + cost_f >= mu:
            break

        # If the forward node has explored, skip it
        if node_f not in f_node:       
            f_node.add(node_f) # Add this node to explored set

        # If the backward node has explored, skip it
        if node_b not in b_node:        
            b_node.add(node_b) # Add this node to explored set

        # if node_f in b_node: # If there is no common nodes, check if a forward or backwaord node in the other search
        #     meeting_node = node_f 
        #     break                      
            
        # if node_b in f_node: # If the backward frontier node has explored in forward process, then it is the meeting node
        #     meeting_node = node_b
        #     break
        
        # if start =='h' and goal == 's':
        #     print("node_f: ", node_f,cost_f)
        #     print("node_b: ",node_b, cost_b)
        #     print("f_node: ",f_node)
        #     print("b_node: ",b_node)
        #     print("Forward Frontier:", forward_frontier)
        #     print("Backward Frontier:", backward_frontier)
        #     print("Forward Visited:", forward_visited)
        #     print("Backward Visited:", backward_visited)
        #     print("Cost Forward:", forward_cost)
        #     print("Cost Backward:", backward_cost)
        #     print("Meeting Node:", meeting_node)
        #     print("mu: ", mu)

        # Expand neighbors of forward frontier
        for neighbor in sorted(graph.neighbors(node_f)):
            if neighbor is None:
                continue
            # Cost so far for a neighbor equal to cost so far for current node 
            # plus current edge cost of current node and this neighbor
            neighbor_cost = forward_cost[node_f] + graph.get_edge_weight(node_f, neighbor) 

            # If this neighbor has not been in explored
            # or if it has been explored, but its cost so far of this path is smaller than explored cost so far
            if neighbor not in forward_cost or neighbor_cost < forward_cost[neighbor]:
                forward_cost[neighbor] = neighbor_cost# Add neighbor and cost so far into forward visited dict
                total_cost = neighbor_cost + heuristic(graph, neighbor, goal) # Calculate estimated total cost
                forward_frontier.append([total_cost, neighbor]) # Add this neighbor and total cost to frontier queue
                forward_visited[neighbor] = node_f

        # Expand neighbors of backward frontier
        for neighbor in sorted(graph.neighbors(node_b)):
            if neighbor is None:
                continue
            # Cost so far for a neighbor equal to cost so far for current node 
            # plus current edge cost of current node and this neighbor
            neighbor_cost = backward_cost[node_b] + graph.get_edge_weight(node_b, neighbor) 

            # If this neighbor has not been in explored
            # or if it has been explored, but its cost so far of this path is smaller than explored cost so far
            if neighbor not in backward_cost or neighbor_cost < backward_cost[neighbor]:

                backward_cost[neighbor] = neighbor_cost # Add neighbor and cost so far into forward visited dict
                total_cost = neighbor_cost + heuristic(graph, neighbor, start) # Calculate estimated total cost
                backward_frontier.append([total_cost, neighbor]) # Add this neighbor and total cost to frontier queue
                backward_visited[neighbor] = node_b

        # Crossover Nodes are from backward frontier union backward explored set, then join forward explored nodes 
        # {crossover} = (backward frontier U backward explored nodes) n forward explored nodes
        if forward_frontier.size()==0:
            f_frontier_set = set()
        else: 
            #Extract nodes from backward frontier
            f_frontier_node = [node for _, node in forward_frontier.queue]
            f_frontier_set = set([node for _,node in f_frontier_node])

        if backward_frontier.size()==0:
            b_frontier_set = set()
        else: 
            #Extract nodes from backward frontier
            b_frontier_node = [node for _, node in backward_frontier.queue]
            b_frontier_set = set([node for _,node in b_frontier_node])
        
        # If forward node meets backward node
        if node_b in f_node or node_f in b_node: 
            c_node = (b_frontier_set.union(b_node)).intersection(f_node) # 48 oout of 400
            for node in c_node:
                total_top = forward_cost[node]+backward_cost[node]
                if total_top < mu: # # if topf + topr < μ
                    mu = total_top # Update μ
                    meeting_node = node # meeting node is the node with lease μ

                       
    # Reconstruct the path
    # If we get the meeting node, add from start to goal
    path = []
    if meeting_node is not None:
        # print(meeting_node)
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
        # if start =='h' and goal == 's':
    # print(path)
    return path
            


