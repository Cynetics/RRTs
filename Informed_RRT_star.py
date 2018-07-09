# -*- coding: utf-8 -*-
"""
Created on Sun Mar 18 02:17:42 2018

@author:  Bryan Cardenas

based on: Informed RRT*: Optimal Sampling-based Path Planning Focused 
          via Direct Sampling of an Admissible Ellipsoidal Heuristic
By: Jonathan Gammell, Siddhartha S. Srinivasa and Timothy D. Barfoot

--------------------------------------------
Modules used: Numpy, Matplotlib, scipy and time

Note that the matplotlib module is used for
simple plotting.
-------------------------------------------
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import time
from RRT_Node import Node
from RRT_Functions import *
from RRT_Plot import *
from RRT_Star_Functions import *

# Seed for constant results
#np.random.seed(7)


#============= Informed RRT* ==================================#
'''
Generate Informed RRT* given an initial value, a maximum amount of 
nodes K. Note that we assume a planning domain with randomly generated
obstacles. 
'''

def informed_RRT_star(x_init, K, step_size,obstacles,goal_area):
    V = [x_init] 
    T = [x_init.coordinates]
    E = []
    path_length = float('inf')
    c_best = float('inf')
    path_found = []
    # The Initialisation of the Ellipse given the theory in the paper 
    # For Informed RRT*
    #------------------
    X_sol = []
    goal_center = Node(goal_area[0][0][0]+0.2,goal_area[0][0][1]+0.2)
    # Euclidean distance to the goal
    c_min = line(goal_center,x_init)
    # the centre of the ellipse
    x_center = np.matrix([[(x_init.x + goal_center.x) / 2.0],[(x_init.y + goal_center.y) / 2.0],[0]])
    a_1 = np.matrix([[(goal_center.x - x_init.x) / c_min],[(goal_center.y - x_init.y) / c_min],[0]])
    # first row of the identity matrix
    id1_t = np.matrix([1.0,0,0])
    M = np.dot(a_1, id1_t)
    # Singular Value Decomposition, S denotes here the Sigma matrix 
    U,S,Vh = np.linalg.svd(M, 1,1)
    # The Rotation Matrix
    C = np.dot(np.dot(U, np.diag([1.0,1.0,  np.linalg.det(U) * np.linalg.det(np.transpose(Vh))])), Vh)
    #------------------
    goal_found = False
    for k in range(K):
        # sample from the ellipse or a uniform distribution
        x_rand = sample(c_best,c_min,x_center,C)
        x_nearest = kd_tree(x_rand,V,T)
        path_new = select_input(x_rand,x_nearest,step_size,partition=True)
        # Check if there is a collision
        if (not collision_rectangle(path_new,obstacles)):
            x_new = path_new[-1]
            V.append(x_new)
            T.append(x_new.coordinates)
            
            x_new.parent = x_nearest
            x_new.cost = x_nearest.cost + line(x_nearest,x_new)
            # nearest neighbours
            X_near = near(V,x_new,T,len(V))
            # Choose a parent from the nearest neighbours
            x_nearest = choose_parent(X_near,x_nearest,x_new,step_size,obstacles)
        
            x_new.parent = x_nearest
            x_new.cost = x_nearest.cost + line(x_nearest,x_new)
            E.append((x_nearest,x_new))
            # Check the tree and rewire if there are improving edges
            E = rewire(X_near,E,x_new,step_size,obstacles)
            # goal check for Informed RRT*
            if collision_rectangle([x_new],goal_area):
                X_sol.append(x_new)
                path_to_goal = trace_path(x_new,[])
                if x_new.cost < path_length:
                    path_length = x_new.cost
                    path_found = path_to_goal
                    c_best = x_new.cost
            # These if statements are for the goal area and cost plotting
            if collision_rectangle([x_new],goal_area) and not goal_found:
                goal_found = True
            elif goal_found:
                path_cost = least_cost(V,goal_area)[1]
                c_best = path_cost
                cost_solutions.append(path_cost)
            else: cost_solutions.append(999)
        else: cost_solutions.append(cost_solutions[-1])
        
    return (V,E,path_found)

previous_diff = 0

# Sample from the ellipsoid by computing the transformation of the ellipsoid
# Given the rotation matrix C
def sample(c_max,c_min,x_center,C):
    global previous_diff
    if c_max < float('inf'):
        previous_diff_temp = c_max**2 - c_min**2
        if previous_diff_temp < 0:
            previous_diff_temp = previous_diff
        previous_diff = previous_diff_temp
        r = [c_max /2.0, np.sqrt(previous_diff)/2.0, np.sqrt(previous_diff)/2.0]
        L = np.diag(r) # The transformation matrix
        x_ball = sample_unit_ball()
        # The random point gets sampled from the heuristic space (ellipsoid)
        random_point = np.dot(np.dot(C,L), x_ball) + x_center
        random_point = Node(random_point[(0,0)], random_point[(1,0)])
        return random_point
    random_point = Node(np.random.uniform(0,10), np.random.uniform(0,10))
    return random_point

  # Sample from a unit ball
def sample_unit_ball():
    x = np.random.random(); y = np.random.random();
    if y < x:
        reference = y
        y = x
        x = reference
    return np.array([[y*np.math.cos(2*np.math.pi*(x/y))], [y*np.math.sin(2*np.math.pi*x/y)], [0]])

#=============== GENERATION ====================================#

def start_generate(step_size,max_nodes,obstacles,origin=Node(5,5),goal=((8,8),0.5,0.5)):
    print("\n[======] - Starting Informed RRT* generation - [======]\n")
    genStart = time.time()
    Informed = informed_RRT_star(origin, max_nodes, step_size,obstacles,goal_area=goal)
    genEnd = time.time()
    path_to_goal,cost = least_cost(Informed[0],goal)
    INFORMED = (Informed[0],Informed[1],path_to_goal)
    print("Amount of nodes Expanded: ", len(Informed[0])) 
    if cost == 999:
        print("No path to the goal was found")
    else:
        print("Best path cost to the goal: ", cost)
    print("\n[======] - Informed RRT* Generation Complete...[======] - ")
    print("\nElapsed time for Informed RRT* Generation: ", genEnd - genStart)
    print("")
    # Plot the vertices, edges, obstacles and the goal
    plot(INFORMED,goal,obstacles,'#00A241')
    print("")

'''
The Informed RRT* is run with epsilon: 0.15, K = 2300
and with a set of 13 obstacles, 5 of which are generated randomly
The initial node originates at (5.5,1.0) and the goal is a square in the
upper right corner
'''
cost_solutions = [999]
K = 1000
obs = 5
# Three chosen obstacles
rect = [((2,2),2.5,1),((5,2),2,1),((7.1,3),1,4)]
# Introduce random obstacles
for i in range(obs):
    rect.append(((np.random.uniform(1,6),np.random.uniform(3.2,10)),
                  np.random.uniform(0.25,1.8),np.random.uniform(0.25,2)))

x_goal = [((6.4,3.5),0.5,0.5)]
x_init = Node(5.5,1)
# epsilon = 0.15, K = 1000, change these parameters for different tree
start_generate(0.15,K,rect,x_init,goal=x_goal)

costs = np.asarray(cost_solutions)
# plot the cost of the best path found
plot_cost(costs,(4,7),K)


