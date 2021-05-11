#!/usr/bin/env python

# import of python modules
from connectivity_pkg.aux_function import edge_generator
import networkx as nx
from networkx.algorithms.approximation import steinertree
from aux_function import *

# Constants
DIST_THRESH = 6

G = nx.Graph()

# vertices = [(1,0), (5,0), (4,0), (-1,0), (0,2), (0, -2), (0,1), (0,-1), (3,3), (0,-3), (0, 7), (0,-9)]

vertices = [(5,0), (0,2), (3, 3), (0,-3), (0, 7)]

elist = edge_generator(vertices, DIST_THRESH)

G.add_weighted_edges_from(elist)

print("built graph {}".format(list(G.nodes)))

terminal_node = [(5,0), (0,7)]
steiner_tree = steinertree.steiner_tree(G, terminal_node)


print("steiner tree node {}".format(list(steiner_tree.nodes)))
print("steiner tree edges {}".format(list(steiner_tree.edges())))
print("terminal node {}".format(set(terminal_node)))
print("connection {}".format(set(steiner_tree.nodes)- set(terminal_node)))