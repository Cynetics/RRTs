
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches


#================ SIMPLE PLOTTING ====================#
  
''' 
Plot the generated tree by connecting
each two nodes in the list of edges using 
a simple plot in matplotlib
'''
def plot(RRT,goal,obstacles):
    ax = plt.axes(xlim=(0,10),ylim=(0,10))
    Nodes, Edges, path = RRT
    for point1,point2 in Edges:
        x1, x2 = point1.x, point2.x
        y1, y2 = point1.y, point2.y
        ax.plot([x1,x2],[y1,y2],'k-',color='#00A241',lw=0.5)
        ax.plot(x2,y2,'bo',markersize=1,color='red')
    
    for point1,point2 in path:

        x1, x2 = point1.x, point2.x
        y1, y2 = point1.y, point2.y
        ax.plot([x1,x2],[y1,y2],'k-',color='red',lw=2.2)
    

    for c, lenght, width in goal:
        ax.add_patch(patches.Rectangle(
            c, 
            lenght, 
            width,
            fill=True,color="green")
        )

    for c, lenght, width in obstacles:
        ax.add_patch(patches.Rectangle(
            c, 
            lenght, 
            width,
            fill=True,color="black")
        )
    
    plt.show()


def plot_cost(costs):
    ax = plt.axes(xlim=(750,K),ylim=(3,5))
    ax.plot(costs,color='#00A241')
    plt.xlabel("Iterations")
    plt.ylabel("Best Solution Cost")
    
    plt.grid()
    plt.show()
    
    np.savetxt('informed_monte_carlo.txt',costs)
    #np.loadtxt('testsave.txt',dtype=float)
