#####################################################
# CS 6601 - Assignment 1͏︌͏󠄁͏︉
# tri_ucs.py͏︌͏󠄁͏︉
#####################################################

# DO NOT ADD OR REMOVE ANY IMPORTS FROM THIS FILE͏︌͏󠄁͏︉
import math
from submission.priority_queue import PriorityQueue

# Credits if any͏︌͏󠄁͏︉
# 1)͏︌͏󠄁͏︉
# 2)͏︌͏󠄁͏︉
# 3)͏︌͏󠄁͏︉

def tridirectional_search(graph, goals):
    """
    Exercise 3: Tridirectional UCS Search
    See README.MD for exercise description.
    Args:
        graph (ExplorableGraph): Undirected graph to search.
        goals (list): Key values for the 3 goals
    Returns:
        The best path as a list from one of the goal nodes (including both of
        the other goal nodes).
    """
  

    if goals[0] == goals[1] and goals[0] == goals[2]:
        return []
    
    # Explored dictionaries have entries of (cost from origin, parent)
    frontier = [PriorityQueue(), PriorityQueue(), PriorityQueue()]
    explored = [{}, {}, {}]
    node_data = [{}, {}, {}]
    # False until the initial meeting of two searches occurs
    meet = False

    path = [[], []]

    # Best node met at so far (state, cost)
    best_meet = None

    for i in range(0,3):
        frontier[i].append((0, goals[i]))
        node_data[i][goals[i]] = (0, None)

    while not meet:
        # Alternate, exploring forward and backward
        for i in range(0,3):
            cost, exploring = frontier[i].pop()
            if exploring in explored[(i + 1) % 3]:
                other_idx = (i + 1) % 3
                cont_idx = (i + 2) % 3
            elif exploring in explored[(i + 2) % 3]:
                other_idx = (i + 2) % 3
                cont_idx = (i + 1) % 3
            elif exploring not in explored[i]:
                
                explored[i][exploring] = node_data[i].pop(exploring)
                # parent_cost = explored[i][exploring][0]
                for state in sorted(graph.neighbors(exploring)):
                    g = graph.get_edge_weight(exploring, state) + explored[i][exploring][0]
                    if state in node_data[i] and node_data[i][state][0] > g:
                        del node_data[i][state]
                    if state not in explored[i] and state not in node_data[i]:
                        node_data[i][state] = (g, exploring)
                        frontier[i].append((g, state))
                continue
            else:
                continue

            # Add exploring to the dict of explored states
            explored[i][exploring] = node_data[i].pop(exploring)
            # Create the union of the frontier and the explored sets for the set we connect with

            explored[other_idx] = merge_explored_frontier(explored[other_idx], frontier[other_idx], node_data[other_idx])
            # Provide the initial meeting point between both explored sets
            best_meet = (exploring, cost + explored[other_idx][exploring][0])
            # Iterate through to see if there is a better meeting point
            best_meet = update_best_meet(best_meet, explored[other_idx], explored[i])
            # Write to the path
            parent = best_meet[0]
            while parent:
                path[0].insert(0, parent)
                parent = explored[i][parent][-1]
            parent = explored[other_idx][best_meet[0]][-1]
            while parent:
                path[0].append(parent)
                parent = explored[other_idx][parent][-1]
            # Union the current explored set with the frontier to aid in finding shortest path for search continuing
            explored[i] = merge_explored_frontier(explored[i], frontier[i], node_data[i])
            meet = True
            break

    # Now continue search for the non-intersected explored set
    # while the state exploring is not in either other stopped explored sets
    while True:
        cost, exploring = frontier[cont_idx].pop()
        # If the state being explored is found in one of the sets, search it to find the lowest cost path
        # Also, search the other explored set to see if it has a lower cost path
        if exploring not in explored[cont_idx]:
            # children = graph[exploring]
            explored[cont_idx][exploring] = node_data[cont_idx].pop(exploring)
            # parent_cost = explored[cont_idx][exploring][0]
            for state in sorted(graph.neighbors(exploring)):
                g = graph.get_edge_weight(exploring, state) + explored[cont_idx][exploring][0]
                if state in node_data[cont_idx] and node_data[cont_idx][state][0] > g:
                    del node_data[cont_idx][state]
                if state not in explored[cont_idx] and state not in node_data[cont_idx]:
                    node_data[cont_idx][state] = (g, exploring)
                    frontier[cont_idx].append((g, state))

        if exploring in explored[(cont_idx + 1) % 3]:
            other_idx = (cont_idx + 1) % 3
            best_meet = (exploring, cost + explored[other_idx][exploring][0])
            best_meet = update_best_meet(best_meet, explored[other_idx], explored[cont_idx])
            best_meet = update_best_meet(best_meet, explored[other_idx], node_data[cont_idx])
            next_best_meet = update_best_meet(best_meet, explored[(cont_idx + 2) % 3], explored[cont_idx])
            next_best_meet = update_best_meet(next_best_meet, explored[(cont_idx + 2) % 3], node_data[cont_idx])
            if next_best_meet[-1] < best_meet[-1]:
                best_meet = next_best_meet
                other_idx = (cont_idx + 2) %3
        elif exploring in explored[(cont_idx + 2) % 3]:
            other_idx = (cont_idx + 2) % 3
            best_meet = (exploring, cost + explored[other_idx][exploring][0])
            best_meet = update_best_meet(best_meet, explored[other_idx], explored[cont_idx])
            continue
        else:
            continue
        parent = best_meet[0]
        while parent:
            path[1].insert(0, parent)
            if(parent in explored[cont_idx]):
                parent = explored[cont_idx][parent][-1]
            else:
                parent = node_data[cont_idx][parent][-1]
        parent = explored[other_idx][best_meet[0]][-1]
        while parent:
            path[1].append(parent)
            if parent in path[0]:
                if parent in explored[(cont_idx + 2) % 3] and parent in explored[(cont_idx + 1) % 3]:
                    if explored[(cont_idx + 2) % 3][parent][0] < explored[(cont_idx + 1) % 3][parent][0]:
                        other_idx = (cont_idx + 2) % 3
                    else:
                        other_idx = (cont_idx + 1) % 3
            parent = explored[other_idx][parent][-1]
        break

    if set(path[1]).issubset(set(path[0])):
        return path[0]
    elif set(path[0]).issubset(set(path[1])):
        return path[1]
    
    if path[0][-1] == path[1][-1]:
        path[1].reverse()
    elif path[0][0] == path[1][-1]:
        # path[1].pop()
        return path[1] + path[0][1:]

    path[1].pop(0)
    for state in path[1]:
        path[0].append(state)
    return path[0]
    


 

# helper for bidirectional_ucs    
def update_best_meet(best_meet, total_explored, this_explored):
    this_cost = float('inf')
    for key in this_explored:
        if key in total_explored:
            this_cost = this_explored[key][0] + total_explored[key][0]
            if this_cost < best_meet[1]:
                best_meet = (key, this_cost)
    return best_meet

# helper for bidirectional_ucs    
def merge_explored_frontier(explored, frontier, node_cost):
    while frontier.size() > 0:
        node = frontier.pop()
        tmp_node = node[1]
        if tmp_node not in explored:
            explored[tmp_node] = node_cost[tmp_node]
    return explored
 





