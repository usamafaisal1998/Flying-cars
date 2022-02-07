# OK this might look a little ugly but...
# Just waiting on eng to install these in the backend!

import pkg_resources
#pkg_resources.require("networkx==2.0")
import networkx as nx

from planning_utils import a_star_graph, distance
import numpy as np
import matplotlib.pyplot as plt
from grid import create_grid_and_edges
import time

plt.rcParams['figure.figsize'] = 6,6

print("start time", time.clock())
# This is the same obstacle data from the previous lesson.
filename = 'colliders.csv'
data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=3)
print(data)

close_start_dist= 9999
close_goal_dist = 9999
start_ne = (25,  100)
#goal_ne = (750., 370.)
goal_ne = (750, 370)

close_start_pt = (9999,9999)
close_goal_pt = (9999,9999)

dist_s = 9999
dist_g = 9999

# Static drone altitude (metres)
drone_altitude = 5

# This is now the routine using Voronoi
grid, edges = create_grid_and_edges(data, drone_altitude)
print(len(edges))

# equivalent to
# plt.imshow(np.flip(grid, 0))
plt.imshow(grid, origin='lower', cmap='Greys')
G = nx.Graph()


def heuristic(n1, n2):
    # TODO: define a heuristic
    return distance(n1,n2)

for e in edges:
    p1 = (int(e[0][0]), int(e[0][1]))
    p2 = (int(e[1][0]), int(e[1][1]))
    dist = distance(p2, p1)
    G.add_edge(p1,p2,weight=dist)
    plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'b-')

    dist_s  = distance(start_ne, p1)
    if dist_s < close_start_dist :
        close_start_dist = dist_s
        close_start_pt = p1

    dist_g  = distance(goal_ne, p1)
    if dist_g < close_goal_dist :
        close_goal_dist = dist_g
        close_goal_pt = p1

print("graph digested", time.clock())
path = a_star_graph(G, heuristic, close_start_pt, close_goal_pt)
print("A* graph found", time.clock())
shortestpath = nx.dijkstra_path(G, close_start_pt, close_goal_pt, 'weight')
print("dijkstra graph found", time.clock())
plt.plot(start_ne[1], start_ne[0], 'rx')
plt.plot(goal_ne[1], goal_ne[0], 'rx')

plt.plot(close_start_pt[1], close_start_pt[0], 'yo')
plt.plot(close_goal_pt[1], close_goal_pt[0], 'yo')

# plot dijkstra path in cyan
for pt in shortestpath :
    plt.plot(pt[1], pt[0], 'g^' )

# plot path in yellow
for p in range(0, len(path[0])-1) :
    pt = path[0][p].next_node()
    plt.plot(pt[1], pt[0], 'yo' )


plt.xlabel('EAST')
plt.ylabel('NORTH')
plt.show()



print("done")