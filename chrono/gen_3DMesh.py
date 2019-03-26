import pychrono as chrono
import pychrono.fea as fea
import math


def gen_3d_cylinder_mesh(length, radius, na=3, nl=2, material=None,
                         shift_x=0, shift_y=0, shift_z=0):
    """
    Generate a 3D cylinder mesh made of na x nl nodes.


    :param length: the length of the cylinder
    :param radius:  the radius of the cylinder
    :param material: the fea.ChMaterialShellReissner material property of the mesh
    :param na: number of nodes in the circumference
    :param nl: number of nodes in the length of the cylinder
    :param shift_x:
    :param shift_y:
    :param shift_z:
    :return: a fea.ChMesh
    """

    # TODO add exceptions

    thickness = 0.1
    ri = 0.06
    ro = 0.1

    mesh = fea.ChMesh()

    # Create the nodes
    matrix_nodes = []
    for i in range(nl):
        line = []
        for j in range(na):

            # Make nodes
            ca = j/na * 2. * math.pi  # current angle in radians
            x = radius * math.cos(ca) + shift_x
            y = radius * math.sin(ca) + shift_y
            z = i/(nl-1) * length + shift_z

            nodepos = chrono.ChVectorD(x, y, z)
            noderot = chrono.ChQuaternionD(chrono.QUNIT)  # TODO maybe rotate
            node = fea.ChNodeFEAxyzrot(chrono.ChFrameD(nodepos, noderot))
            line.append(node)

            # Add node to mesh
            mesh.AddNode(node)
            node.SetMass(0)

        matrix_nodes.append(line)

    # Create the elements

    for i in range(nl-1):
        print(i)
        for j in range(na):

            # Make elements
            elem = fea.ChElementShellReissner4()

            if j == na-1:
                # Last elem
                elem.SetNodes(
                    matrix_nodes[i+1][j],  # top right
                    matrix_nodes[i][j],  # top left
                    matrix_nodes[i][0],  # bottom left
                    matrix_nodes[i + 1][0]  # bottom right
                )
            else:
                elem.SetNodes(
                    matrix_nodes[i+1][j],  # top right
                    matrix_nodes[i][j],  # top left
                    matrix_nodes[i][j+1],  # bottom left
                    matrix_nodes[i+1][j+1]  # bottom right
                )
            if material:
                elem.AddLayer(thickness, 0 * chrono.CH_C_DEG_TO_RAD, material)
                elem.SetAlphaDamp(0.0)

            mesh.AddElement(elem)

    # Connect last nodes

    return mesh
