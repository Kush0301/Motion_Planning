#!/usr/bin/env python
# coding: utf-8

# In[2]:


#!/usr/bin/env python
import osmnx as ox
import networkx as nx
import queue
import math
import priority_dict


map_graph = ox.graph_from_place('Berkeley, California', network_type='drive')
origin = ox.get_nearest_node(map_graph, (37.8743, -122.277))
destination = list(map_graph.nodes())[-1]




# Dijkstra's Search
def dijkstras_search(origin_key, goal_key, graph):
    open_queue = priority_dict.priority_dict({})
    closed_dict = {}
    predecessors = {}
    open_queue[origin_key] = 0.0
    goal_found = False
    while (open_queue):
        u,ucost=open_queue.pop_smallest()
        if u==goal_key:
            goal_found=True
            break
        for edge in graph.out_edges([u], data=True):
            v=edge[1]
            if v in closed_dict:
                continue
            uvcost=edge[2]['length']
            if v in open_queue.keys():
                if ucost+uvcost<open_queue[v]:
                    open_queue[v]=ucost+uvcost
                    predecessors[v]=u
            else:
                open_queue[v]=ucost+uvcost
                predecessors[v]=u
        closed_dict[u]=1
            
    if not goal_found:
        raise ValueError("Goal not found in search.")

    return get_path(origin_key, goal_key, predecessors)                


def get_path(origin_key, goal_key, predecessors):
    key = goal_key
    path = [goal_key]
    
    while (key != origin_key):
        key = predecessors[key]
        path.insert(0, key)
        
    return path


#Distance heuristic for A*
def distance_heuristic(state_key, goal_key, node_data):
    n1 = node_data[state_key]
    n2 = node_data[goal_key]

    long1 = n1['x']*math.pi/180.0
    lat1 = n1['y']*math.pi/180.0
    long2 = n2['x']*math.pi/180.0
    lat2 = n2['y']*math.pi/180.0
    
    r = 6371000
    x1 = r*math.cos(lat1)*math.cos(long1)
    y1 = r*math.cos(lat1)*math.sin(long1)
    z1 = r*math.sin(lat1)

    x2 = r*math.cos(lat2)*math.cos(long2)
    y2 = r*math.cos(lat2)*math.sin(long2)
    z2 = r*math.sin(lat2)

    d = ((x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2)**0.5
    
    return d


#A* search
def a_star_search(origin_key, goal_key, graph):

    open_queue = priority_dict.priority_dict({})
    closed_dict = {}
    predecessors = {}
    costs = {}
    node_data = graph.nodes(True)
    costs[origin_key] = 0.0
    open_queue[origin_key] = distance_heuristic(origin_key, goal_key, node_data)
    goal_found = False
    
    while (open_queue):
        u,u_h=open_queue.pop_smallest()
        ucost=costs[u]
        if u==goal_key:
            goal_found=True
            break
        for edge in graph.out_edges([u], data=True):
            v=edge[1]
            if v in closed_dict:
                continue
            uvcost=edge[2]['length']
            if v in open_queue.keys():
                if ucost+uvcost+distance_heuristic(v,goal_key,node_data)<open_queue[v]:
                    open_queue[v]=ucost+uvcost+distance_heuristic(v,goal_key,node_data)
                    costs[v]=ucost+uvcost
                    predecessors[v]=u
            else:
                open_queue[v]=ucost+uvcost+distance_heuristic(v,goal_key,node_data)
                costs[v]=ucost+uvcost
                predecessors[v]=u
        closed_dict[u]=1

    if not goal_found:
        raise ValueError("Goal not found in search.")
    return get_path(origin_key, goal_key, predecessors) 

#Shortest Path
shortest_path = nx.shortest_path(map_graph, origin, destination, weight='length')
fig, ax = ox.plot_graph_route(map_graph, shortest_path)

#Using Dijkstras
path = dijkstras_search(origin, destination, map_graph)        
fig, ax = ox.plot_graph_route(map_graph, path) 


#Using A*
path = a_star_search(origin, destination, map_graph)        
fig, ax = ox.plot_graph_route(map_graph, path) 


# In[ ]:





# In[ ]:




