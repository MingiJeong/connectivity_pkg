#!/usr/bin/env python

# import of python modules
import math
import numpy as np
from random import randrange, uniform

from numpy.lib.function_base import append

# Constant
# requirement of assignment about 10 x 10 area
X_coord_min = -5
X_coord_max = 5
Y_coord_min = -5
Y_coord_max = 5
CLOSE_DIST_THRESH = 3 # ranomness close distance

def none_in_dict(d):
    """
    function to check whether there is "None" in dict value
    Args:
        dictionary data
    Returns:
        boolean to indicate whether None is existing or not
    """
    for key, value in d.items():
        if value is None:
            return True
    return False

def false_in_dict(d):
    """
    function to check whether there is "False" in dict value
    Args:
        dictionary data
    Returns:
        boolean to indicate whether False is existing or not
    """
    for key, value in d.items():
        if not value:
            return True
    return False


def random_point_generator(point_total=3):
    """
    function to generate random waypoints according to the task requirement
    Args:
        the number of point_total to be generated (default 3)
    Returns:
        generated points in list format [x1, y1, x2, y2, x3, y3 ...]
    """
    generated_point = []
    # uniform gives you a floating-point value
    while len(generated_point) < point_total * 2:
        float_rand_x = uniform(X_coord_min, X_coord_max)
        float_rand_y = uniform(Y_coord_min, Y_coord_max)
        
        current_point = closeness_check(generated_point, [float_rand_x, float_rand_y])
        if current_point is not None:
            generated_point.extend(current_point)
        
    return generated_point

def closeness_check(generated_list, current_point):
    """
    function to check distance between the previously generated waypoin and current potential waypoint
    Args:
        - generated points in list format [x1, y1, x2, y2, x3, y3 ...]
        - a potential point to be added
    Returns: 
        - if current point meet the distance requirement, it is added.
    """
    if len(generated_list) == 0:
        # current point is the first one; just skip
        pass
    
    else:
        for i in range(len(generated_list)/2):
            # (i, i+1)
            point_to_compare = generated_list[i*2:i*2+2]
            if distance_calculator(current_point, point_to_compare) < CLOSE_DIST_THRESH:
                return
        
    return current_point

def edge_generator(vertices, dist_thresh):
    """
    function to return valid edge from initial input vertices in consideration of distance threshhold
    Args:
        verticies: list of vertices in format [(x1, y1), (x2, y2), ...]
        dist_thresh: user-defined threshhold
    Returns:
        edge_list [(vertex1, vertex2, cost),...] meeting the distance requirement
        this will be used to form a network graph by G.add_weighted_edges_from() 
    """
    i = 0
    edge_list = list()
    while i < len(vertices) - 1:
        j = i+1
        while j < len(vertices):
            calculated_distance = distance_calculator(vertices[i], vertices[j])
            if  calculated_distance <= dist_thresh:
                edge_list.append((vertices[i], vertices[j], calculated_distance))
            j += 1
        i += 1
    # print(edge_list)
    return edge_list


def distance_calculator(point1, point2):
    """
    function to return Euclidean distance between two coordinates between point1 and point2
    Args:
        point1: list [x1,y1]
        point2: list [x2,y2] 
    Returns:
        Euclidean distance
    """
    return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)

def check_vector_angle(vec1, vec2):
    """
    check angle between two vectors by using vec 1 as base.
    Args: 
        vec1, vec2 format as np.array([x,y]) 1x2 vector
    Returns:
        angle (in radian) between the vectors abs 0 to pi (+: if coincide with check vector, -: if not)
    """

    # inner dot calculation
    inner_dot = np.dot(vec1, vec2)

    # vector length calculation
    vec1_len = np.linalg.norm(vec1)
    vec2_len = np.linalg.norm(vec2)

    # between angle calculation
    between_angle = math.acos(round(np.asscalar(inner_dot / (vec1_len * vec2_len)),6))

    # check vector (assuming "between angle" is correct, we get rotation vector of vec1 based on that angle)
    # length normalization and magnitude reflection
    check_vector = [vec1[0] * math.cos(between_angle) - vec1[1] * math.sin(between_angle),
                        vec1[0] * math.sin(between_angle) + vec1[1] * math.cos(between_angle)]
    check_vector_normalized = [check_vector[i] / vec1_len for i in range(len(check_vector))]
    vec2_normalized = [vec2[i] / vec2_len for i in range(len(check_vector))]

    # check if the vector is the same as what we want
    # note that 0.1 is just a threshhold for comparison
    if abs(check_vector_normalized[0] - vec2_normalized[0]) < 0.1 and abs(check_vector_normalized[1] - vec2_normalized[1]) < 0.1:
        return between_angle
    return -between_angle