import pychrono as chrono
import pychrono.fea as fea
import math
import numpy as np
import gen_cylinder
import chrono_utils as tool


class SleeveShellReissner:
    """
    A sleeve defined as a 3D cylinder with elastical properties
    """

    MIN_NA = 3  # Minimum number of nodes in the circumference
    MIN_NL = 2  # Minimum number of nodes in the length
    na = 3
    nl = 2
    nodes = None  # List of nodes
    fea_nodes = None
    edges = None  # list of edges
    elements = None  # List of elements
    mesh = None

    def __init__(self, length, radius, na=MIN_NA, nl=MIN_NL, material=None,
                 node_mass=10., sleeve_thickness=0.1, alphadamp=0.1,
                 shift_x=0, shift_y=0, shift_z=0):
        """
        Initialize a 3D cylinder mesh made of na x nl nodes.
        This forms the skeleton of the sleeve.

        :param length: the length of the sleeve
        :param radius: the radius of the sleeve
        :param na: number of nodes in the circumference
        :param nl: number of nodes in the length
        :param material: a fea.ChMaterialShellReissner material property of the mesh
        :param node_mass: mass of the node
        :param sleeve_thickness: Thickness of the sleeve
        :param alphadamp This is the Rayleigh "alpha" for the stiffness-proportional damping.
        This assumes damping forces as F=alpha*[Km]*v where [Km] is the stiffness matrix (material part,
        i.e.excluding geometric stiffness) and v is a vector of node speeds. Usually,
        alpha in the range 0.0 - 0.1 Note that the mass-proportional term of classical Rayleigh damping is not
        supported.
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

        self.na = na
        self.nl = nl

        # Create the nodes and edges
        self.nodes, self.edges = gen_cylinder.gen_cylinder(radius, length, na, nl,
                                                           shift_x, shift_y, shift_z)

        # Create the mesh made of nodes and elements
        self.mesh = fea.ChMesh()

        # Create FEA nodes
        self.fea_nodes = []
        for n in self.nodes:
            nodepos = tool.make_ChVectorD(n)
            noderot = chrono.ChQuaternionD(chrono.QUNIT)
            node = fea.ChNodeFEAxyzrot(chrono.ChFrameD(nodepos, noderot))
            node.SetMass(node_mass)
            self.fea_nodes.append(node)
            self.mesh.AddNode(node)

        # Create FEA elements
        self.elements = []
        for i in range(nl - 1):
            for j in range(na):

                # Make elements
                elem = fea.ChElementShellReissner4()

                if j == na - 1:
                    # Connect last nodes to the first ones
                    elem.SetNodes(
                        self.fea_nodes[(i+1)*na],  # top right
                        self.fea_nodes[i*na],  # top left
                        self.fea_nodes[j+i*na],  # bottom left
                        self.fea_nodes[j+(i+1)*na]  # bottom right
                    )
                else:
                    elem.SetNodes(
                        self.fea_nodes[j+1+(i+1)*na],  # top right
                        self.fea_nodes[j+1+i*na],  # top left
                        self.fea_nodes[j+i*na],  # bottom left
                        self.fea_nodes[j+(i+1)*na]  # bottom right
                    )
                if material:
                    elem.AddLayer(sleeve_thickness, 0 * chrono.CH_C_DEG_TO_RAD, material)
                    elem.SetAlphaDamp(alphadamp)

                self.mesh.AddElement(elem)
                self.elements.append(elem)

    def get_mesh(self):
        return self.mesh

    def fix_extremities(self, body, system):
        """
        Fix the extremities of the sleeve to the left and right side of the body

        :param body: a ChBody where the nodes of the sleeve will be fixed
        :param system:
        :return:
        """
        for fn in self.fea_nodes[:self.na] + self.fea_nodes[self.na*(self.nl-1):]:
            constr = chrono.ChLinkMateGeneric()
            constr.Initialize(fn, body, False, fn.Frame(), fn.Frame())
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
                self.fea_nodes[j*self.na+i].SetForce(force_vector)

    def update_nodes(self):
        for i, fn in enumerate(self.fea_nodes):
            pos = eval(str(fn.GetPos()))
            self.nodes[i] = pos

    def release(self):
        """
        Release the forces of the nodes by setting the force to 0
        :return:
        """
        for fn in self.fea_nodes:
            fn.SetForce(chrono.ChVectorD(0., 0., 0.))

    def get_sd_nodes(self, previous_nodes):
        dist = []
        for pn, cn in zip(previous_nodes, self.nodes):
            dist.append(np.linalg.norm(pn-cn))
        #print(sum(dist))
        return sum(dist)/len(previous_nodes)


