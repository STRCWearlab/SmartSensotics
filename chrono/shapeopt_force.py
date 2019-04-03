import geo

def shapeopt_force(nodes_apx, edges_apx, nodes_t, edges_t):
    """

    :param nodes_apx:
    :param edges_apx:
    :param nodes_t:
    :param edges_t:
    :return: the coordinates of the optimised nodes
    """

    # Pre-compute the target edge lengths
    edges_t_len = geo.edgelen_all(nodes_t, edges_t)

    # Parameters
    nu = .2  # Convergence speed
    it = 3  # Total iterations

    nodes_opt = nodes_apx
    fvall = None

    # Convergence iteration
    for i in range(it):

        # force vectors of all the nodes to be optimized
        fvall = geo.node_force_all(nodes_opt, edges_apx, edges_t_len)

        # Convergence speed
        fvall *= nu

        # Update nodes position
        nodes_opt += fvall

    return nodes_opt, fvall
