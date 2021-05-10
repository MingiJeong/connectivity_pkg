#!/usr/bin/env python

import networkx as nx

G = nx.Graph()

vertices = [(1,5), (4,0), (-4,0)]

elist = [('a', 'b', 5.0), ('b', 'c', 3.0), ('a', 'c', 1.0), ('c', 'd', 7.3)]
G.add_weighted_edges_from(elist)

print(list(G.nodes))