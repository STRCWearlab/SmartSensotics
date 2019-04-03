import math
import numpy as np

def gen_cylinder(radius, length, na, nl,
                 shift_x=0, shift_y=0, shift_z=0):
    """
    Generate a 3D cylinder made of na nodes in the circumference and nl nodes in the length

    The nodes are numbered first in the circumference then in the length.

    :param radius: radius of the cylinder
    :param length: length of the cylinder
    :param na: number of nodes in the circumference
    :param nl: number of nodes in the length
    :return: (nodes, edges) where 'nodes' is a matrix of Nx3 where N is the number of nodes with x,y,z coordinates and
    'edges' is a matrix of Nx4 indicating the neighbourhood for each node of 'nodes'. Set -1 if no neighbour.
    an edge is defined as [left_node, next_angle_node, previous_angle_node, right_node]
    """
    # Handle unrealistic values
    if na < 3:
        raise ValueError(f"Number of nodes in the circumference must be >= 3.")
    if nl < 2:
        raise ValueError(f"Number of nodes in the length must be >= 2.")

    # Nodes
    nodes = []
    edges = []
    print(length)
    for i in range(nl):
        cl = length/(nl-1)*i + shift_z  # current length

        for j in range(na):

            # Make nodes
            ca = j / na * 2. * math.pi  # current angle in radians
            x = radius * math.cos(ca) + shift_x
            y = radius * math.sin(ca) + shift_y

            nodes.append([x, y, cl])

            # Make edges
            # [left_node, next_angle_node, right_node, previous_angle_node]
            e = [-1, -1, -1, -1]

            # Link left
            if i > 0:
                e[0] = na * (i - 1) + j
            # link right
            if i < nl-1:
                e[2] = na * (i + 1) + j
            # link to previous angle
            if j > 0:
                e[3] = na * i + j - 1
            else:
                e[3] = na * (i + 1) - 1
            # link to next angle
            if j < na-1:
                e[1] = na * i + j + 1
            else:
                e[1] = na * i

            edges.append(e)

            #print(f"Node[{i},{j}] {na*i+j} (ca={ca}, cl={cl}) edges: {e[0]} {e[1]} {e[2]} {e[3]}")

    return np.array(nodes), np.array(edges)
