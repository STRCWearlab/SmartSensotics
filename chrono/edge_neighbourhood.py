def edge_neighbourhood(neighbours, nidx, a, l, na, nl):
    """
    Assign the edge neighbourhood to the edge

    #       #       #
      \     |     /
        6   0   5
          \ | /
    #-- 2 --#-- 3 --#
          / | \
        4   1   7
      /     |     \
    #       #       #

    :param neighbours:
    :param nidx:
    :param a:
    :param l:
    :param na:
    :param nl:
    :return:
    """

    # Init edge
    e = [-1] * neighbours

    # Link left
    if l > 0 and neighbours > 2:
        e[2] = nidx - na

        # Link to ang_prev - left
        if neighbours > 4:
            if a > 0:
                e[4] = nidx - na - 1
            else:
                e[4] = nidx - 1
        # Link to next - left
        if neighbours > 6:
            if a < na - 1:
                e[6] = nidx - na + 1
            else:
                e[6] = nidx + 1 - na - na

    # link right
    if l < nl - 1 and neighbours > 2:
        e[3] = nidx + na

        # Link to ang_next - right
        if neighbours > 4:
            if a < na - 1:
                e[5] = nidx + na + 1
            else:
                e[5] = nidx + 1
        # Link to prev - right
        if neighbours > 6:
            if a > 0:
                e[7] = nidx + na - 1
            else:
                e[7] = nidx + na + na - 1

    # link to previous angle
    if a > 0:
        e[1] = nidx - 1
    else:
        e[1] = nidx + (na - 1)

    # link to next angle
    if a < na - 1:
        e[0] = nidx + 1
    else:
        e[0] = nidx - (na - 1)

    return e