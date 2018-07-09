
import numpy as np
from RRT_Node import Node
from RRT_Functions import *

'''
This file containts the three procedures relevant to RRT* and
Informed RRT*: The (choose) Parent procedure, the Rewire procedure
and the function returning the set of nearest neighbours to a node.
'''

'''
The choose Parent procedure choose a parent for the new node
given a ball radius (local search)
'''
def choose_parent(X_near,x_nearest,x_new,step_size,obstacle):
    for x_near in X_near:
        # if obstacle free
        c_prime = x_near.cost + line(x_near,x_new)
        path_new = select_input(x_new,x_near,step_size,partition=True)
        if (not collision_rectangle(path_new,obstacle)):
            if c_prime < x_new.cost:
                x_new.cost = c_prime
                x_nearest = x_near
    return x_nearest

'''
The rewire procedure checks if a better path exists through
the new node and rewires the tree accordingly.
'''
def rewire(X_near,E,x_new,step_size,obstacle):
    for x_near in X_near:
        path_new = select_input(x_new,x_near,step_size,partition=True)
        if (not collision_rectangle(path_new,obstacle)):
            if x_near.cost > x_new.cost + line(x_near,x_new):
                x_parent = x_near.parent
                x_near.parent = x_new
                x_near.cost = x_new.cost + line(x_near,x_new)
                E.remove((x_parent,x_near))
                E.append((x_new,x_near))
    return E
    
'''
The near procedure returns the nodes within a ball radius
defined by x_new
'''
def near(V,x_new,node_coordinates,length_V):
    eta = 0.4 # max radius
    gamma = 50
    zheta = gamma * (np.log(length_V) / length_V) # surface of the ball  
    t = np.power((gamma / zheta) * (np.log(length_V) / length_V),1/2)
    r = min(t,eta)
    near_set = kd_tree_radius(x_new,V,node_coordinates,r)

    return near_set

