import pychrono as chrono


def make_ChVectorD(v):
    if len(v) != 3:
        raise ValueError('Vector v must have a length of 3 for x,y,z coordinates')
    return chrono.ChVectorD(v[0], v[1], v[2])
