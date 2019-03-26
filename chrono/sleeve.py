import pychrono as chrono
import pychrono.fea as fea
import math


class Sleeve:
    """
    A sleeve defined as a 3D cylinder with elastical properties
    """

    MIN_NA = 3  # Minimum number of nodes in the circumference
    MIN_NL = 2  # Minimum number of nodes in the length
    nodes = []  # Matrix of nodes
    elements = []  # Matrix of elements
    mesh = None

    def __init__(self, length, radius, na=MIN_NA, nl=MIN_NL, material=None,
                 shift_x=0, shift_y=0, shift_z=0):
        """
        Initialize a 3D cylinder mesh made of na x nl nodes.
        This forms the skeleton of the sleeve.


        :param length: the length of the sleeve
        :param radius: the radius of the sleeve
        :param na: number of nodes in the circumference
        :param nl: number of nodes in the length
        :param material: a fea.ChMaterialShellReissner material property of the mesh
        :param shift_x: the sleeve displacement from the origin on x axis
        :param shift_y: the sleeve displacement from the origin on y axis
        :param shift_z: the sleeve displacement from the origin on z axis
        """

        # Handle unrealistic values
        if na < 3:
            raise ValueError(f"Number of nodes in the circumference"
                             f" must be >= {self.MIN_NA}.")
        if nl < 2:
            raise ValueError(f"Number of nodes in the length must be >= {self.MIN_NL}.")

        sleeve_thickness = 0.05
        ri = 0.06
        ro = 0.1

        # Create the mesh made of nodes and elements
        self.mesh = fea.ChMesh()

        # Nodes
        for i in range(nl):
            line = []
            for j in range(na):
                # Make nodes
                ca = j / na * 2. * math.pi  # current angle in radians
                x = radius * math.cos(ca) + shift_x
                y = radius * math.sin(ca) + shift_y
                z = i / (nl - 1) * length + shift_z

                nodepos = chrono.ChVectorD(x, y, z)
                noderot = chrono.ChQuaternionD(chrono.QUNIT)  # TODO maybe rotate
                node = fea.ChNodeFEAxyzrot(chrono.ChFrameD(nodepos, noderot))
                line.append(node)

                # Add node to mesh
                node.SetMass(1000)
                self.mesh.AddNode(node)

            self.nodes.append(line)

        # Elements
        for i in range(nl - 1):
            line = []
            for j in range(na):

                # Make elements
                elem = fea.ChElementShellReissner4()

                if j == na - 1:
                    # Connect last nodes to the first ones
                    elem.SetNodes(
                        self.nodes[i + 1][j],  # top right
                        self.nodes[i][j],  # top left
                        self.nodes[i][0],  # bottom left
                        self.nodes[i + 1][0]  # bottom right
                    )
                else:
                    elem.SetNodes(
                        self.nodes[i + 1][j],  # top right
                        self.nodes[i][j],  # top left
                        self.nodes[i][j + 1],  # bottom left
                        self.nodes[i + 1][j + 1]  # bottom right
                    )
                if material:
                    elem.AddLayer(sleeve_thickness, 0 * chrono.CH_C_DEG_TO_RAD, material)
                    elem.SetAlphaDamp(0.0)

                line.append(elem)
                self.mesh.AddElement(elem)

            self.elements.append(line)

    def getMesh(self):
        return self.mesh
