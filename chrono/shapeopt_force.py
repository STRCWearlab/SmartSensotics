import geo


def shapeopt_force(nodes_apx, edges_apx, nodes_t, edges_t, neighbours):
    """

    :param nodes_apx: Nodes of inferred shape
    :param edges_apx: Edges of inferred shape
    :param nodes_t: Nodes of target shape
    :param edges_t: Edges of target shape
    :param neighbours: number of neighbours
    :return: the coordinates of the inferred nodes
    """

    # Pre-compute the target edge lengths
    edges_t_len = geo.edgelen_all(nodes_t, edges_t, neighbours)

    # Parameters
    nu = .2  # Convergence speed
    it = 2  # Total iterations

    nodes_opt = nodes_apx
    fvall = None

    # Convergence iteration
    for i in range(it):
        # force vectors of all the nodes to be optimized
        fvall = geo.node_force_all(nodes_opt, edges_apx, edges_t_len, neighbours)

        # Convergence speed
        fvall *= nu

        # Update nodes position
        nodes_opt += fvall

    return nodes_opt, fvall
