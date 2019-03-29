import pychrono as chrono
import pychrono.fea as fea
import math


class SleeveShellReissner:
    """
    A sleeve defined as a 3D cylinder with elastical properties
    """

    MIN_NA = 3  # Minimum number of nodes in the circumference
    MIN_NL = 2  # Minimum number of nodes in the length
    na = 3
    nl = 2
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

        sleeve_thickness = 0.1
        # This is the Rayleigh "alpha" for the stiffness-proportional damping.
        # This assumes damping forces as F=alpha*[Km]*v where [Km] is the stiffness matrix (material part,
        # i.e.excluding geometric stiffness) and v is a vector of node speeds. Usually,
        # alpha in the range 0.0 - 0.1 Note that the mass-proportional term of classical Rayleigh damping is not
        # supported.
        alphadamp = 0.1  #

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

                # node = fea.ChNodeFEAxyzD(nodepos)
                line.append(node)
                # Add node to mesh
                node.SetMass(10.0)
                self.mesh.AddNode(node)

            self.nodes.append(line)

        # Elements
        for i in range(nl - 1):
            line = []
            for j in range(na):

                # Make elements
                elem = fea.ChElementShellReissner4()
                # elem = fea.ChElementShellANCF()

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
                    elem.SetAlphaDamp(alphadamp)

                line.append(elem)
                self.mesh.AddElement(elem)

            self.elements.append(line)

        self.na = na
        self.nl = nl

    def get_mesh(self):
        return self.mesh

    def fix_extremities(self, body, system):
        """
        Fix the extremities of the sleeve to the left and/or right side of the body

        :param body: a ChBody where the nodes of the sleeve will be fixed
        :param left: if True, fix the left part of the sleeve to the left part of the body
        :param right: if True, fix the right part of the sleeve to the right part of the body
        :return:
        """
        for i in range(self.na):
            for j in [0, self.nl-1]:
                constr = chrono.ChLinkMateGeneric()
                node = self.nodes[j][i]
                constr.Initialize(node, body, False, node.Frame(), node.Frame())
                system.Add(constr)
                constr.SetConstrainedCoords(True, True, True, True, True, True)

    def extend(self, force):
        """
        Apply a load to each element of the mesh from the inside to the outside of the sleeve.

        :param force: the force applied towards the outside of the sleeve in Newton
        :return: void
        """
        for i in range(self.na):
            ca = i / self.na * 2. * math.pi  # current angle in radians
            force_vector = chrono.ChVectorD(math.cos(ca) * force,  # x
                                            math.sin(ca) * force,  # y
                                            0  # z
                                            )
            for j in range(self.nl):
                # Apply a force towards the outside of the sleeve
                self.nodes[j][i].SetForce(force_vector)

    def release(self):
        """
        Release the forces of the nodes by setting the force to 0
        :return:
        """
        for i in range(self.na):
            for j in range(self.nl):
                self.nodes[j][i].SetForce(chrono.ChVectorD(0., 0., 0.))
