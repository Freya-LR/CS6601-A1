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
    # raise NotImplementedError
    goal_u,goal_v,goal_z = goals
    if goal_u == goal_v  == goal_z: # If three nodes are the same
        return []
    # print(goals)
    # False until the initial meeting of two searches occurs
    meet = False
    link = [[], []]
    # meeting node (node, cost)
    meeting_node = None
    # Initialize g cost
    g_cost_u = {goal_u:0}
    g_cost_z = {goal_z:0}
    g_cost_v = {goal_v:0}
    # Initialize 3 frontiers
    frontier_u,frontier_v,frontier_z = PriorityQueue(),PriorityQueue(),PriorityQueue() # initialized frontier_joint queue
    frontier_u.append([heuristic(graph, goal_u,goal_v)+g_cost_u[goal_u],g_cost_u[goal_u],goal_u]) # append the start node goal_u
    u_visited = {goal_u: (0,None)} # node: (cost so far , its current)
    # For goal v
    frontier_v.append([heuristic(graph, goal_u,goal_v)+g_cost_v[goal_v],g_cost_v[goal_v],goal_v])
    v_visited = {goal_v: (0,None)}   
    # For goal z
    frontier_z.append([heuristic(graph, goal_u,goal_z)+g_cost_z[goal_z],g_cost_z[goal_z],goal_z])
    z_visited = {goal_z: (0,None)}

    # Initialize the joint frontier_joint, reached_list list and visited list
    frontier_joint= [frontier_u, frontier_v, frontier_z]
    reached_list = [{},{},{}]
    visited_list = [u_visited, v_visited, z_visited]
    
    while not meet:
        # Process from each goal
        for i,frontier in enumerate(frontier_joint):
            # if frontier_joint.size()>0: 
            f_cost, g_cost, node = frontier.pop()

            if node in reached_list[(i + 1) % 3]:# If the node in the second reached_list set    
                j,k = (i + 1) % 3, (i + 2) % 3 # j: contains meeting node, k: no meeting node
            # Determine the other corresponding frontier_joints
            elif node in reached_list[(i + 2) % 3]: # If the node in the third reached_list set
                j,k = (i + 2) % 3, (i + 1) % 3
            # Expand from node i
            elif node not in reached_list[i]:    
                reached_list[i][node] = visited_list[i].pop(node) # If the lowest cost node is not in i reached_list list, add it

                for neighbor in sorted(graph.neighbors(node)):
                    edge_cost = graph.get_edge_weight(node, neighbor) 
                    g_neighbor = g_cost+edge_cost
                    h_neighbor = heuristic(graph,neighbor, node)
                    f_neighbor = g_neighbor+h_neighbor

                    if neighbor not in reached_list[i] and neighbor not in visited_list[i]:
                        visited_list[i][neighbor] = (g_neighbor, node)
                        frontier_joint[i].append((f_neighbor,g_neighbor, neighbor))

                continue
            else:
                continue

            # if node found in reached_list set j, Move the node to reached_list list
            reached_list[i][node] = visited_list[i].pop(node)
            # update reached_list list j, let all visited nodes are in reached_list set
            reached_list[j] = bi_frontier(reached_list[j], frontier_joint[j], visited_list[j])
            # Initialize meeting node, cost is sum of two reached_list sets
            meeting_node = (node, g_cost + reached_list[j][node][0])
            # link[0] = buid_link(meeting_node, reached_list[j],reached_list[k])
            meeting_node = bi_meet(meeting_node, reached_list[j], reached_list[i])
            # Write to the link
            current = meeting_node[0]
            while current:
                link[0].insert(0, current)
                current = reached_list[i][current][-1]
            current = reached_list[j][meeting_node[0]][-1]
            while current:
                link[0].append(current)
                current = reached_list[j][current][-1]

            # update reached_list list i, let all visited nodes are in reached_list set
            reached_list[i] = bi_frontier(reached_list[i], frontier_joint[i], visited_list[i])
            meet = True
            break

    while True:
        # For the frontier_joint that hasn't intersacted with
        
        f_cost, g_cost, node = frontier_joint[k].pop()
        # If the neighbor being explored is found in one of the sets, search it to find the lowest cost link
        # Also, search the other explored set to see if it has a lower cost link
        if node not in reached_list[k]:
            reached_list[k][node] = visited_list[k].pop(node)
            for neighbor in sorted(graph.neighbors(node)):
                edge_cost = graph.get_edge_weight(node, neighbor) 
                g_neighbor = g_cost+edge_cost
                h_neighbor = heuristic(graph,neighbor, node)
                f_neighbor = g_neighbor+h_neighbor
                if neighbor not in reached_list[k] and neighbor not in visited_list[k]:
                    visited_list[k][neighbor] = (g_neighbor, node)
                    frontier_joint[k].append((f_neighbor,g_neighbor, neighbor))

        if node in reached_list[(k + 1) % 3]:
            j = (k + 1) % 3
            meeting_node = (node, g_cost + reached_list[j][node][0])
            # update the meeting node to get lower cost from reached_list list j and k
            meeting_node = bi_meet(meeting_node, reached_list[j], reached_list[k])
            # update the meeting node to get lower cost from reached_list list j and visited list k
            meeting_node = bi_meet(meeting_node, reached_list[j], visited_list[k])
            # update the next meeting node to get lower cost from the third reached_list list and reached_list list k
            next_meeting_node = bi_meet(meeting_node, reached_list[(k + 2) % 3], reached_list[k])
            # update the next meeting node to get lower cost from the third reached_list list and visited list k
            next_meeting_node = bi_meet(next_meeting_node, reached_list[(k + 2) % 3], visited_list[k])
            
            if next_meeting_node[-1] < meeting_node[-1]: # if the cost of next meeting node is less than the current one 
                meeting_node = next_meeting_node
                j = (k + 2) %3

        elif node in reached_list[(k + 2) % 3]:
            j = (k + 2) % 3
            meeting_node = (node, g_cost + reached_list[j][node][0])
            meeting_node = bi_meet(meeting_node, reached_list[j], reached_list[k])

        #     continue
        # else:
        #     continue

        current = meeting_node[0]
        while current:
            link[1].insert(0, current)
            if(current in reached_list[k]):                 
                current = reached_list[k][current][-1]               
            elif current in visited_list[k]:
                current = visited_list[k][current][-1]
            else:
                break
        current = reached_list[j][meeting_node[0]][-1]         
        while current:
            link[1].append(current)

            current = reached_list[j][current][-1]
        break
    # if goals == ['a','u','l']:
    #     print(link[0], link[1])
    return combine_link(link[0],link[1])



def bi_meet(meeting_node, reached_list_1, reached_list_2):
    """if a node is in both reached_list set, return the lowest cost one
    """
    for node in reached_list_1:
        if node in reached_list_2:
            mu = reached_list_2[node][0] + reached_list_1[node][0]
            if  mu < meeting_node[1]:
                meeting_node = (node, mu)
    return meeting_node
# helper for bidirectional_ucs    
def bi_frontier(reached_list, frontier_joint, node_cost):
    while frontier_joint.size() > 0:
        node = frontier_joint.pop()
        tmp_node = node[1]
        if tmp_node not in reached_list:
            if tmp_node not in node_cost:
                node_cost[tmp_node] = float('inf')
            reached_list[tmp_node] = node_cost[tmp_node]
    return reached_list
def combine_link(link1,link2):
    # If link1 contains link2, return link1 as final link
    if set(link2).issubset(set(link1)):
        return link1
    elif set(link1).issubset(set(link2)):
        return link2
    
    # Combine two links
    if link1[-1] == link2[-1]: # If last node of link 1 is the same as last node of link 2
        link2.reverse()
        return link1+link2[1:]
    elif link1[0] == link2[-1]: # If first node of link 1 is the same as last node of link 2
        return link2 + link1[1:]
    
    link2.pop(0)
    link1.extend(link2)
    return link1