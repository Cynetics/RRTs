import numpy as np

# The node class includes information for each vertex in the tree
# The important ones: The coordinates, Cost and Parent
class Node:
    cost = 0
    parent = None
    coordinates = [0,0]
    x, y = 0,0
    def __init__(self,xc,yc,cost=0,parent=None):
        self.coordinates = np.asarray([xc,yc])
        self.x = xc
        self.y = yc
        self.cost = cost
        self.parent = parent