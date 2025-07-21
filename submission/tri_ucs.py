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

def tridirectional_search(graph, goals) -> list:
    """
    Exercise 3: Tridirectional UCS Search

    See README.MD for exercise description.

    Args:
        graph (ExplorableGraph): Undirected graph to search.
        goals (list): Key values for the 3 goals

    Returns:
        The best link as a list from one of the goal nodes (including both of
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

    frontier_u,frontier_v,frontier_z = PriorityQueue(),PriorityQueue(),PriorityQueue() # initialized frontier_joint queue
    frontier_u.append([0,goal_u]) # append the start node goal_u
    u_visited = {goal_u: (0,None)} # node: (cost so far , its current)
    # For goal v
    frontier_v.append([0,goal_v])
    v_visited = {goal_v: (0,None)}   
    # For goal z
    frontier_z.append([0,goal_z])
    z_visited = {goal_z: (0,None)}

    # Initialize the joint frontier_joint, reached_list list and visited list
    frontier_joint= [frontier_u, frontier_v, frontier_z]
    reached_list = [{},{},{}]
    visited_list = [u_visited, v_visited, z_visited]
    
    while not meet:
        # Process from each goal
        for i,frontier in enumerate(frontier_joint):
            # if frontier_joint.size()>0: 
            cost, node = frontier.pop()
            # if goals == ['s', 't', 'o']:
            #     print(cost, node)
            if node in reached_list[(i + 1) % 3]:# If the node in the second reached_list set    
                j,k = (i + 1) % 3, (i + 2) % 3 # j: contains meeting node, k: no meeting node
            # Determine the other corresponding frontier_joints
            elif node in reached_list[(i + 2) % 3]: # If the node in the third reached_list set
                j,k = (i + 2) % 3, (i + 1) % 3
            # Expand from node i
            elif node not in reached_list[i]:    
                reached_list[i][node] = visited_list[i].pop(node) # If the lowest cost node is not in i reached_list list, add it
                # if goals == ['s', 't', 'o']:
                #     print(reached_list[i][node])
                for neighbor in sorted(graph.neighbors(node)):
                    neighbor_cost = graph.get_edge_weight(node, neighbor) + reached_list[i][node][0]
                    if neighbor in visited_list[i] and neighbor_cost<visited_list[i][neighbor][0] :
                        del visited_list[i][neighbor] # if the node is already in reached_list list, or the new cost is smaller than existing one, delete it
                    if neighbor not in reached_list[i] and neighbor not in visited_list[i]:
                        visited_list[i][neighbor] = (neighbor_cost, node)
                        frontier_joint[i].append((neighbor_cost, neighbor))
                    # if goals == ['s', 't', 'o']:
                    #     print(reached_list[i])
                continue
            else:
                continue

            # if node found in reached_list set j, Move the node to reached_list list
            reached_list[i][node] = visited_list[i].pop(node)
            # update reached_list list j, let all visited nodes are in reached_list set
            reached_list[j] = bi_frontier(reached_list[j], frontier_joint[j], visited_list[j])
            # Initialize meeting node, cost is sum of two reached_list sets
            meeting_node = (node, cost + reached_list[j][node][0])
            # link[0] = buid_link(meeting_node, reached_list[j],reached_list[k])
            meeting_node = bi_meet(meeting_node, reached_list[j], reached_list[i])
            # Write to the link
            current = meeting_node[0]
            # if goals == ['b', 'c', 'd']:
            #         print(current)
            while current:
                #     print(meeting_node)
                #     print(link[0], link[1])
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
        
        cost, node = frontier_joint[k].pop()
        # If the neighbor being explored is found in one of the sets, search it to find the lowest cost link
        # Also, search the other explored set to see if it has a lower cost link
        if node not in reached_list[k]:
            reached_list[k][node] = visited_list[k].pop(node)
            for neighbor in sorted(graph.neighbors(node)):
                neighbor_cost = graph.get_edge_weight(node, neighbor) + reached_list[k][node][0]
                if neighbor in visited_list[k] and neighbor_cost< visited_list[k][neighbor][0] :
                    del visited_list[k][neighbor] # if the node is already in reached_list list, or the new cost is smaller than existing one, delete it
                if neighbor not in reached_list[k] and neighbor not in visited_list[k]:
                    visited_list[k][neighbor] = (neighbor_cost, node)
                    frontier_joint[k].append((neighbor_cost, neighbor))

        if node in reached_list[(k + 1) % 3]:
            j = (k + 1) % 3
            meeting_node = (node, cost + reached_list[j][node][0])
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
            # if goals == ['s', 't', 'o']:
            #     print(meeting_node)
        elif node in reached_list[(k + 2) % 3]:
            j = (k + 2) % 3
            meeting_node = (node, cost + reached_list[j][node][0])
            meeting_node = bi_meet(meeting_node, reached_list[j], reached_list[k])

            continue
        else:
            continue


        current = meeting_node[0]
        while current:
            link[1].insert(0, current)
            if(current in reached_list[k]):                 
                current = reached_list[k][current][-1] # none               
            else:
                current = visited_list[k][current][-1]
        current = reached_list[j][meeting_node[0]][-1]  # o
        
        while current:
            link[1].append(current)
            if current in link[0]:
                if current in reached_list[(k + 2) % 3] and current in reached_list[(k + 1) % 3]:
                    if reached_list[(k + 2) % 3][current][0] < reached_list[(k + 1) % 3][current][0]:
                        j = (k + 2) % 3
                    else:
                        j = (k + 1) % 3
            current = reached_list[j][current][-1]


        break

    return combine_link(link[0],link[1])

def bi_meet(meeting_node, reached_list_1, reached_list_2):
    """if a node is in both reached_list set, return the lowest cost one
    """
    for node in reached_list_2:
        if node in reached_list_1:
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