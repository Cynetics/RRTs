# -*- coding: utf-8 -*-
"""
Created on Sun Mar 18 02:17:42 2018

@author:  Bryan Cardenas

based on: RRT: Rapidly Exploring Random Trees:
                A new tool for path planning
By: Steven M. LaValle
--------------------------------------------
Modules used: Numpy, Matplotlib, scipy, time

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


#============= RRT ==================================#
'''
Generate RRT given an initial value, a maximum amount of 
nodes K. Note that we assume a planning domain with randomly generated
obstacles. 
'''

def RRT(x_init, K, step_size,obstacles,goal_area):
    V = [x_init] 
    T = [x_init.coordinates]
    E = []
    path_found = []
    goal_found = False
    for k in range(K):
        # sample from the ellipse or a uniform distribution
        x_rand = Node(np.random.uniform(0,10), np.random.uniform(0,10))
        x_nearest = kd_tree(x_rand,V,T)
        path_new = select_input(x_rand,x_nearest,step_size,partition=True)
        # Check if there is a collision
        if (not collision_rectangle(path_new,obstacles)):
            x_new = path_new[-1]
            V.append(x_new)
            T.append(x_new.coordinates)     
            x_new.parent = x_nearest
            x_new.cost = x_nearest.cost + line(x_nearest,x_new)
            E.append((x_nearest,x_new))
            
            # These if statements are used for the goal area and cost plotting
            if collision_rectangle([x_new],goal_area) and not goal_found:
                goal_found = True
            elif goal_found:
                path_cost = least_cost(V,goal_area)[1]
                cost_solutions.append(path_cost)
            else: cost_solutions.append(999)
        else: cost_solutions.append(cost_solutions[-1])
        
    return (V,E,path_found)

#=============== GENERATION ====================================#

def start_generate(step_size,max_nodes,obstacles,origin=Node(5,5),goal=((8,8),0.5,0.5)):
    print("\n[======] - Starting RRT generation - [======]\n")
    genStart = time.time()
    RRt = RRT(origin, max_nodes, step_size,obstacles,goal_area=goal)
    genEnd = time.time()
    path_to_goal, cost = least_cost(RRt[0],goal)
    RRt = (RRt[0],RRt[1],path_to_goal)
    print("Amount of nodes Expanded: ", len(RRt[0])) 
    if cost == 999:
        print("No path to the goal was found")
    else:
        print("Best path cost to the goal: ", cost)
    print("\n[======] - RRT Generation Complete...[======] - ")
    print("\nElapsed time for RRT Generation: ", genEnd - genStart)
    print("")
    # Plot the vertices, edges, obstacles and the goal
    plot(RRt,goal,obstacles,'black')
    print("")

'''
The RRT is run with epsilon: 0.15, K = 2300
and with a set of 13 obstacles, 5 of which are generated randomly
The initial node originates at (5.5,1.0) and the goal is a square in the
upper right corner
'''
cost_solutions = [999]
K = 1000
obs = 5
goal_start_point = (6.4,3.5)

# Three chosen obstacles
rect = [((2,2),2.5,1),((5,2),2,1),((7.1,3),1,4)]
# Introduce random obstacles
for i in range(obs):
    rect.append(((np.random.uniform(1,6),np.random.uniform(3.2,10)),
                  np.random.uniform(0.25,1.8),np.random.uniform(0.25,2)))

x_goal = [(goal_start_point,0.5,0.5)]
x_init = Node(5.5,1)
# [==========================================]
# epsilon = 0.15, K = 1000, change these parameters for different tree
start_generate(0.15,K,rect,x_init,goal=x_goal)
# [=========================================]
costs = np.asarray(cost_solutions)
plot_cost(costs,(4,7),K)
# plot the cost of the best path found

