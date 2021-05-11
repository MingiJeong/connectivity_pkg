#!/usr/bin/env python

# import of python modules
import networkx as nx
import aux_function
import random
from networkx.algorithms.approximation import steinertree

# custom modules
from connectivity_pkg.aux_function import edge_generator

# Constants
DIST_THRESH = 8 # for graph building


def network_generator(random_generate=False):
    """
    function to generate network based on steiner tree approximation
    Args: 
        random_generate boolean flag
    Returns: 
        terminal_nodes, connection_nodes, entire_nodes
    """

    # graph initialization
    G = nx.Graph()

    # randomness
    if not random_generate:
        vertices = [(5,0), (0,2), (3, 4), (0,-3), (-1, 7)]
        terminal_node = [vertices[0], vertices[-1]]
        
    else:
        vertices = network_node_converter_2d(aux_function.random_point_generator(5))
        
        # selection of terminal node (2ea)
        terminal_node_lottery = list(range(0,len(vertices)))
        random.shuffle(terminal_node_lottery)
        terminal_node = [vertices[terminal_node_lottery[0]], vertices[terminal_node_lottery[1]]]
        
    # eldge list by meeting the distance threshold
    elist = aux_function.edge_generator(vertices, DIST_THRESH)
    # graph building with edges
    G.add_weighted_edges_from(elist)


    # steiner tree approximation
    steiner_tree = steinertree.steiner_tree(G, terminal_node)
    connection_node = list(set(steiner_tree.nodes)- set(terminal_node))

    # checking with print
    print("built graph {}".format(list(G.nodes)))
    print("steiner tree node {}".format(list(steiner_tree.nodes)))
    print("steiner tree edges {}".format(list(steiner_tree.edges())))
    print("terminal node {}".format(set(terminal_node)))
    print("connection {}".format(set(steiner_tree.nodes)- set(terminal_node)))

    return network_node_converter_linear(terminal_node), network_node_converter_linear(connection_node), network_node_converter_linear(list(steiner_tree.nodes))


def network_node_converter_2d(node_list):
    """
    function to convert 1D node list into 2D node list
    Args:
        node_list with format [x1, y1, x2, y2, ...]
    Returns:
        output with format [(x1, y1), (x2, y2), ...]
    """
    output = []
    for i in range(len(node_list)/2):
        output.append((node_list[2*i], node_list[2*i+1]))
    return output

def network_node_converter_linear(node_list):
    """
    function to convert 2D node list into 1D node list
    Args:
        node_list with format [(x1, y1), (x2, y2), ...]
    Returns:
        output with format [x1, y1, x2, y2, ...]
    """
    output = []
    for node in node_list:
        for component in node:
            output.append(component)
    return output