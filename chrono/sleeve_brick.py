import pychrono as chrono
import pychrono.fea as fea
import math


class SleeveBrick:
    """
    A sleeve defined as a 3D cylinder with elastical properties
    Element material made of bricks
    """

    MIN_BA = 3  # Minimum number of nodes in the circumference
    MIN_BL = 2  # Minimum number of nodes in the length
    na = 3
    nl = 2
    nodes_inner = []  # Matrix of nodes
    nodes_outer = []  # Matrix of nodes
    elements = []  # Matrix of elements
    mesh = None

    def __init__(self, length, radius, sleeve_thickness, ba=MIN_BA, bl=MIN_BL,
                 material=None, shift_x=0, shift_y=0, shift_z=0):
        """
        Initialize a 3D cylinder mesh made of na x nl nodes.
        This forms the skeleton of the sleeve.


        :param length: the length of the sleeve
        :param radius: the radius of the sleeve
        :param ba: number of bricks in the circumference
        :param bl: number of bricks in the length
        :param shift_x: the sleeve displacement from the origin on x axis
        :param shift_y: the sleeve displacement from the origin on y axis
        :param shift_z: the sleeve displacement from the origin on z axis
        """

        # Handle unrealistic values
        if ba < 3:
            raise ValueError(f"Number of nodes in the circumference"
                             f" must be >= {self.MIN_BA}.")
        if bl < 2:
            raise ValueError(f"Number of nodes in the length must be >= {self.MIN_BL}.")

        # This is the Rayleigh "alpha" for the stiffness-proportional damping.
        # This assumes damping forces as F=alpha*[Km]*v where [Km] is the stiffness matrix (material part,
        # i.e.excluding geometric stiffness) and v is a vector of node speeds. Usually,
        # alpha in the range 0.0 - 0.1 Note that the mass-proportional term of classical Rayleigh damping is not
        # supported.
        alphadamp = 0.1  #

        # Create the mesh made of nodes and elements
        self.mesh = fea.ChMesh()

        # Nodes
        for i in range(bl+1):
            line_inner = []
            line_outer = []
            for j in range(ba):
                # Make nodes
                ca = j / ba * 2. * math.pi  # current angle in radians
                xi = radius * math.cos(ca) + shift_x
                yi = radius * math.sin(ca) + shift_y
                xo = (radius+sleeve_thickness) * math.cos(ca) + shift_x
                yo = (radius+sleeve_thickness) * math.sin(ca) + shift_y
                z = i / (bl) * length + shift_z

                nodepos_i = chrono.ChVectorD(xi, yi, z)
                nodepos_o = chrono.ChVectorD(xo, yo, z)
                node_i = fea.ChNodeFEAxyz(nodepos_i)
                node_o = fea.ChNodeFEAxyz(nodepos_o)

                line_inner.append(node_i)
                line_outer.append(node_o)
                # Add node to mesh
                node_i.SetMass(10.0)
                node_o.SetMass(10.0)
                self.mesh.AddNode(node_i)
                self.mesh.AddNode(node_o)

            self.nodes_inner.append(line_inner)
            self.nodes_outer.append(line_outer)

        # Brick Elements
        for i in range(bl):
            line = []
            for j in range(ba):

                # Make the brick elements
                elem = fea.ChElementBrick()

                if j == ba - 1:
                    # Connect last nodes to the first ones
                    elem.SetNodes(
                        self.nodes_inner[i][j],  # top left
                        self.nodes_inner[i+1][j],  # bottom left
                        self.nodes_inner[i+1][0],  # bottom right
                        self.nodes_inner[i][0],  # top right
                        self.nodes_outer[i][j],  # top left
                        self.nodes_outer[i+1][j],  # bottom left
                        self.nodes_outer[i+1][0],  # bottom right
                        self.nodes_outer[i][0],  # top right
                    )
                    # elem.SetNodes(
                    #     self.nodes_inner[i+1][j],  # bottom left
                    #     self.nodes_inner[i][j],  # top left
                    #     self.nodes_inner[i][0],  # top right
                    #     self.nodes_inner[i+1][0],  # bottom right
                    #     self.nodes_outer[i+1][j],  # bottom left
                    #     self.nodes_outer[i][j],  # top left
                    #     self.nodes_outer[i][0],  # top right
                    #     self.nodes_outer[i+1][0],  # bottom right
                    # )
                    # elem.SetNodes(
                    #     self.nodes_inner[i + 1][j],  # top right
                    #     self.nodes_inner[i][j],  # top left
                    #     self.nodes_inner[i][0],  # bottom left
                    #     self.nodes_inner[i + 1][0],  # bottom right
                    #     self.nodes_outer[i + 1][j],  # top right
                    #     self.nodes_outer[i][j],  # top left
                    #     self.nodes_outer[i][0],  # bottom left
                    #     self.nodes_outer[i + 1][0]  # bottom right
                    # )
                else:
                    elem.SetNodes(
                        self.nodes_inner[i][j],  # top left
                        self.nodes_inner[i+1][j],  # bottom left
                        self.nodes_inner[i+1][j+1],  # bottom right
                        self.nodes_inner[i][j+1],  # top right
                        self.nodes_outer[i][j],  # top left
                        self.nodes_outer[i+1][j],  # bottom left
                        self.nodes_outer[i+1][j+1],  # bottom right
                        self.nodes_outer[i][j+1],  # top right
                    )
                    # elem.SetNodes(
                    #     self.nodes_inner[i + 1][j],  # bottom left
                    #     self.nodes_inner[i][j],  # top left
                    #     self.nodes_inner[i][j + 1],  # top right
                    #     self.nodes_inner[i + 1][j + 1],  # bottom right
                    #     self.nodes_outer[i + 1][j],  # bottom left
                    #     self.nodes_outer[i][j],  # top left
                    #     self.nodes_outer[i][j + 1],  # top right
                    #     self.nodes_outer[i + 1][j + 1],  # bottom right
                    # )
                    # elem.SetNodes(
                    #     self.nodes_inner[i + 1][j],  # top right
                    #     self.nodes_inner[i][j],  # top left
                    #     self.nodes_inner[i][j + 1],  # bottom left
                    #     self.nodes_inner[i + 1][j + 1],  # bottom right
                    #     self.nodes_outer[i + 1][j],  # top right
                    #     self.nodes_outer[i][j],  # top left
                    #     self.nodes_outer[i][j + 1],  # bottom left
                    #     self.nodes_outer[i + 1][j + 1]  # bottom right
                    # )
                #elem.SetMaterial(material)  # ISSUE not working
                # TODO mooney-rivlin elasticity, otherwise it is linear
                #elem.SetMooneyRivlin(True)
                #elem.SetMRCoefficients(0.3339, -3.37e-4)

                line.append(elem)
                self.mesh.AddElement(elem)

            self.elements.append(line)

        self.ba = ba
        self.bl = bl

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
