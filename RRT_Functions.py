import numpy as np
from scipy import spatial
from Node import Node
import sys

'''
This file contains the functions used by all the three RRT
algorithms. This mainly contains the functions responsible
for computing the paths, the path costs, or the nearest
neighbour procedure (kd-tree). It also contains the Select
input that chooses a new node minimising the distance to the 
random node. Furthermore, a naive collision check procedure
is used.

'''


# Get the path given a starting point in the tree
def trace_path(child_node,path):
    if child_node.cost == 0:
        path.append((child_node,child_node))
        return path
    else:
        parent = child_node.parent
        path.append((parent,child_node))
        return trace_path(child_node.parent,path)

# compute the cost of a path
def cost_of_path(path):
    cost = 0
    for edge in path:
        cost += line(edge[0],edge[1])
    return cost

# Search for the least cost path maintained by the tree
def least_cost(V,goal_area):
  path_to_goal = []
  best_path = []
  cost = 999
  for node in V:
      if collision_rectangle([node],goal_area):
          path_to_goal = trace_path(node,[])
          goal_cost = cost_of_path(path_to_goal)
          if goal_cost <= cost:
              cost = goal_cost
              best_path = path_to_goal
  return (best_path,cost)  

# the distance between the two given points
def line(x_near, x_new):
    return np.linalg.norm(x_near.coordinates - x_new.coordinates)
        
'''
Iterate through the list with edges and compute
the distance between the nodes, take the closest edge
to the random node x_rand. THIS IS A NAIVE IMPLEMENTATION
'''
def nearest_neighbour(x_rand, T, forbidden = []):
    nearest = (None, sys.maxsize)
    for point in T:
        #print(point)
        p0, p1 = point.x, point.y
        x0, x1 = x_rand.x, x_rand.y
        # We do not want the random point to be a point in T
        # Or in the list of nodes we forbade due to collisions
        if x0 == p0 and x1 == p1: continue
        distance = np.linalg.norm(x_rand.coordinates - point.coordinates)
        # keep track of the nearest
        if distance < nearest[1]:
            nearest = (point,distance)
    return nearest[0]

# KD-tree for efficient nearest neighbour search in O(log(n))
def kd_tree(node,node_list, node_coordinates):
    tree = spatial.KDTree(np.asarray(node_coordinates))
    query_point = node.coordinates
    found_neighbour = tree.query(query_point)
    return node_list[found_neighbour[1]]

# The kd-tree giving a set of nodes nearest to a vertex
def kd_tree_radius(node,node_list,node_coordinates,ball_radius):
    tree = spatial.KDTree(np.asarray(node_coordinates))
    query_point = node.coordinates
    found_neighbours = tree.query_ball_point(query_point,ball_radius)
    # return the list with nearest neighbours given a ball radius
    return [node_list[i] for i in found_neighbours]
    

'''
Given the nearest point and the random point
return a valid new point (no collisions)
'''
def select_input(x_rand, x_near, step_size, partition=False):
    if line(x_rand,x_near) <= step_size:
        return [x_rand]
    v = [x_rand.x- x_near.x, x_rand.y - x_near.y]
    # Normalize
    u = v / np.linalg.norm(v)
    dotted_path = []
    dots = 30
    if partition:
        for i in range(dots):
            new_point = x_near.coordinates + (step_size / dots) * u * i
            dotted_path.append(Node(new_point[0],new_point[1]))
    else: 
        # Get the new node by making one step in the right direction
        new_node = x_near.coordinates + step_size * u
        return [Node(new_node[0],new_node[1])]
    return dotted_path



'''
The collision check mechanism takes a list of obstacles and list of points to
check if the points lie within the obstacles
'''
def collision_rectangle(potential_points, obstacles):
    #print(obstacles)
    for ((rectX, rectY), width, height) in obstacles:
        for point in potential_points:
            x, y = point.x, point.y
            if (x <= rectX + width and x >= rectX) and \
                (y <= rectY + height and y >= rectY):
                    return True
    return False
