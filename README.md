# RRTs
A Bachelor Thesis impelmentation of RRT, RRT* and Informed RRT*. The Thesis_Images folder contains all the images that were made in Photoshop. These images are free for use.
# Configuration
The Algorithms are implemented in Python using the numpy, matplotlib and scipy libraries. The base-line algorithms (RRT, RRT* and Informed RRT*) are implemented in Informed_RRT_Star.py, RRT_Star.py and RRT.py respectively. The functions that these algorithms share are separated in other files.

# Installation
The numpy, matplotlib and scipy libraries must be installed before use as these are extensively used
in the implementation. The versions used are 1.11.3 1.5.3, 1.1.0 respectively. The Python version used was 3.7.0.

# Instructions
To use the files, simply run RRT.py or RRT_Star.py or Informed_RRT_Star.py. The number of iterations is initially set to 1000. After a few seconds matplotlib shows the run of the algorithm and the cost of the best path is plotted against the iterations. Note that, since the algorithms are probabilistic, a few runs may be necessary to find the path in the three implementations. It is advised to open the files and play around with the parameters, especially the step-size (epsilon) parameter and the number of iterations.

# Files
Informed_RRT_Star.py,
Node.py,
plot_RRT.py,
RRT.py,
RRT_Functions.py,
RRT_Node.py,
RRT_Plot.py,
RRT_Star.py,
RRT_Star_Functions.py,
RRT_Star_Procedure.py,
Thesis_Images.

# Known Problems
The obstacles are generated randomly (The number generated is set to find initially) and one could spawn on the goal area. This could make it difficult for the tree to find the goal area. 

# Credits
This code is made by Bryan Cardenas Guevara for the Artificial Intelligence Bachelor thesis at Utrecht University, The Netherlands. 
