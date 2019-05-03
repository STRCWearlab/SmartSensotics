import numpy as np
import geo

def shape_energy(nodes_inf, edges_inf, edges_t_len, neighbours):
    """
    Shape optimisation based on energy. Energy is defined
    as the sum of squared differences of the edges lentgh between
    inffered and target edges

    :param nodes_inf: Nodes of inferred shape
    :param edges_inf: Edges of inferred shape
    :param edges_t_len: Target edge lengths
    :param neighbours: Number of neighbours
    :return: The energy defined as in description
    """
    # Reshape as a nx3 matrix of node coordinates
    nodes_inf = np.reshape(nodes_inf, (int(len(nodes_inf)/3), 3))

    edges_inf_len = geo.edgelen_all(nodes_inf, edges_inf, neighbours)

    # Difference squared
    e_diff = np.square(np.array(edges_t_len)-np.array(edges_inf_len))

    # Keep difference squared only for non-nan
    e_diff = e_diff[~np.isnan(e_diff)]

    # Convert to energy
    return sum(e_diff)