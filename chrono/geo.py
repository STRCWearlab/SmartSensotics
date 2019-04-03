import numpy as np


def node_force(nodes, edges, idx, rest_len):
    """
    Compute the force vector for the idx node in nodes given the target edges and
    target edges lengths.
    :param nodes: the nodes of the shape to approximate to compute the force from
    :param edges: the edges of the shape to approximate
    :param idx: the index of the current node
    :param rest_len: the length of the target edges.
    :return:
    """
    # Compute the current length of the four neighbouring edges of node idx
    cl = edgelen(nodes, edges, idx)

    # Initiate force vector
    fv = np.zeros(3)

    # Iterate the four edges
    for i in range(4):

        # If the edge exists
        if cl[i]:
            # fa: force amplitude
            # fa > 0 means current edge is longer than target
            fa = cl[i] - rest_len[i]

            # fd: force direction (unit vector)
            # force direction: move towards neighbour
            fd = nodes[edges[idx][i]] - nodes[idx]

            # force direction must be normalised
            fd = fd / np.linalg.norm(fd)

            # force vector is unit vector multiplied by amplitude
            fv = fv + fd * fa

    return fv


def node_force_all(nodes, edges, edge_target_len):
    """
    Compute the displacement force on node in a pseudo-physical spring model

    Spring model:
    - If length of current node to neighbour is longer than rest: be attracted to neighbor
    - If length of current node to neighbour is shorter than rest: be repulsed to neighbor
    :param nodes: All nodes of shape to optimise
    :param edges: All edges of shape to optimise
    :param edge_target_len: Length of edges at rest (target)
    :return:
    """
    fvall = [[0] * 3] * len(nodes)
    for i in range(len(nodes)):
        fv = node_force(nodes, edges, i, edge_target_len[i])
        fvall[i] = fv
    return np.array(fvall)


def edgelen(nodes, edges, idx):
    """
    Compute the lenght of the four edges of the node at index idx
    :param nodes: A Nx
    :param edges:
    :param idx:
    :return:
    """
    l = [None] * 4
    cp = nodes[idx]  # Coordinate of current node

    # Iterate over all edges of the current node
    for i in range(4):
        # If edge exists
        if edges[idx][i] > 0:
            # Coordinate of neighbour node:
            cn = nodes[edges[idx][i]]

            # Edge length
            l[i] = np.linalg.norm(cp - cn)
    return np.array(l)


def edgelen_all(nodes, edges):
    """
    Compute the length of all the nodes edges

    :param nodes:
    :param edges:
    :return: Nx4 matrix where N is the node index with the four lengths of the node edges.
    None indicates that an edge does not exist.
    """
    len_all = []
    for i in range(len(nodes)):
        l = edgelen(nodes, edges, i)
        len_all.append(l)

    return np.array(len_all)
