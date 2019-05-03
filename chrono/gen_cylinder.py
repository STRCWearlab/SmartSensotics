import math
import numpy as np
from edge_neighbourhood import edge_neighbourhood


def gen_cylinder(radius, length, na, nl, neighbours,
                 shift_x=0., shift_y=0., shift_z=0.):
    """
    Generate a 3D cylinder made of na nodes in the circumference and nl nodes in the length

    The nodes are numbered first in the circumference then in the length.

    :param radius: radius of the cylinder
    :param length: length of the cylinder
    :param na: number of nodes in the circumference
    :param nl: number of nodes in the length
    :param neighbours: number of neighbours (2,4,6,8)
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
    nidx = 0
    for i in range(nl):
        cl = length / (nl - 1) * i + shift_z  # current length

        for j in range(na):

            # Make nodes
            ca = j / na * 2. * math.pi  # current angle in radians
            x = radius * math.cos(ca) + shift_x
            y = radius * math.sin(ca) + shift_y

            nodes.append([x, y, cl])

            # Make edges
            e = edge_neighbourhood(neighbours, nidx, j, i, na, nl)
            edges.append(e)
            nidx += 1

            # print(f"Node[{i},{j}] {na*i+j} (ca={ca}, cl={cl}) edges: {e[0]} {e[1]} {e[2]} {e[3]}")

    return np.array(nodes), np.array(edges)


def downsample(from_nodes, from_edges, from_na, from_nl,
               to_na, to_nl):
    """
    Downsample the number of nodes from to to.
    !!! Call update_nodes() to be sure to have the positions updated
    :param from_nodes:
    :param from_edges:
    :param from_na:
    :param from_nl:
    :param to_na:
    :param to_nl:
    :param na: Number of nodes desired in the angle
    :param nl: Number of nodes desired in the length (number of rings)
    :return: the nodes and edges structure
    """
    l_jump = int(math.ceil(from_nl/to_nl))
    a_jump = int(math.ceil(from_na/to_na))
    # Build the mask that tells which nodes to keep
    mask = [False] * (from_na * from_nl)
    for i in range(from_na):
        for j in range(from_nl):
            idx = i + j * from_nl
            mask[idx] = j % l_jump == 0 and i % a_jump == 0

    # TODO add last nodes if target nodes are missing

    # Update neighborhood
    _, to_edges = gen_cylinder(10,10,to_na, to_nl)
    # to_edges = from_edges.copy()
    # for i in range(from_na):
    #     for j in range(from_nl):
    #         idx = i+j*from_nl
    #         if mask[idx] and i + a_jump < from_na:
    #
    #             # Update left neighbor
    #             if from_edges[idx][0] >= 0:
    #                 to_edges[idx][0] = i+(j-l_jump)*from_nl
    #
    #             # Update right neighbor
    #             if from_edges[idx][2] >= 0:
    #                 to_edges[idx][2] = i+(j+l_jump)*from_nl
    #
    #             # Update next angle neighbor
    #             to_edges[idx][1] = i+a_jump+j*from_nl
    #             to_edges[idx][1] = i+a_jump+j*from_nl
    #
    #             # Update previous angle neighbor
    #             to_edges[idx][3] = i-a_jump+j*from_nl

    # Filter nodes and edges with mask
    to_nodes = np.array([n for n, c in zip(from_nodes, mask) if c])
    #to_edges = [e for e, c in zip(to_edges, mask) if c]

    #print('# target nodes, len nodes, len edges', to_na*to_nl, len(to_nodes), len(to_edges))
    #print(to_nodes)
    #print(to_edges)
    return to_nodes, to_edges
