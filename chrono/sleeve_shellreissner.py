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
    nodes = []  # List of nodes
    fea_nodes = []
    edges = None  # list of edges
    elements = None  # List of elements
    mesh = None
    length = -1
    radius = -1
    shift_x = 0
    shift_y = 0

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
        self.length = length
        self.radius = radius
        self.shift_x = shift_x
        self.shift_y = shift_y

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
                        self.fea_nodes[(i + 1) * na],  # top right
                        self.fea_nodes[i * na],  # top left
                        self.fea_nodes[j + i * na],  # bottom left
                        self.fea_nodes[j + (i + 1) * na]  # bottom right
                    )
                else:
                    elem.SetNodes(
                        self.fea_nodes[j + 1 + (i + 1) * na],  # top right
                        self.fea_nodes[j + 1 + i * na],  # top left
                        self.fea_nodes[j + i * na],  # bottom left
                        self.fea_nodes[j + (i + 1) * na]  # bottom right
                    )
                if material:
                    elem.AddLayer(sleeve_thickness, 0 * chrono.CH_C_DEG_TO_RAD, material)
                    elem.SetAlphaDamp(alphadamp)

                self.mesh.AddElement(elem)
                self.elements.append(elem)

    def get_mesh(self):
        return self.mesh

    def freeze(self):
        for fn in self.fea_nodes:
            fn.SetFixed(True)

    def unfreeze(self):
        for fn in self.fea_nodes:
            fn.SetFixed(False)

    def fix_extremities(self, body, system):
        """
        Fix the extremities of the sleeve to the left and right side of the body

        :param body: a ChBody where the nodes of the sleeve will be fixed
        :param system:
        :return:
        """
        for fn in self.fea_nodes[:self.na] + self.fea_nodes[self.na * (self.nl - 1):]:
            constr = chrono.ChLinkMateGeneric()
            constr.Initialize(fn, body, False, fn.Frame(), fn.Frame())
            system.Add(constr)
            constr.SetConstrainedCoords(True, True, True, True, True, True)

    def move(self, fns, ns, radius):
        i = 0
        for fn, n in zip(fns, ns):
            ca = i / self.na * 2. * math.pi  # current angle in radians
            x = radius * math.cos(ca) + self.shift_x
            y = radius * math.sin(ca) + self.shift_y
            n = [x, y, n[2]]
            fn.SetPos(tool.make_ChVectorD(n))
            i += 1

    def move_to_extremities(self, radius_left, radius_right):
        self.move(self.fea_nodes[:self.na],
                  self.nodes[:self.na],
                  radius_left)
        self.move(self.fea_nodes[self.na * (self.nl - 1):],
                  self.nodes[self.na * (self.nl - 1):],
                  radius_right)

    def expand(self, force):
        """
        Apply a load to each element of the mesh from the inside to the outside of the sleeve.

        :param force: the force applied towards the outside of the sleeve in Newton
        :return: void
        """
        for i in range(self.na):
            ca = i / self.na * 2. * math.pi  # current angle in radians
            fv_x = math.cos(ca) * force
            fv_y = math.sin(ca) * force
            for j in range(self.nl):
                fv_z = 0.
                force_vector = chrono.ChVectorD(fv_x, fv_y, fv_z)
                # Apply a force towards the outside of the sleeve
                self.fea_nodes[j * self.na + i].SetForce(force_vector)

    def update_nodes(self):
        for i, fn in enumerate(self.fea_nodes):
            pos = eval(str(fn.GetPos()))
            self.nodes[i] = pos

    def get_nnodes(self):
        """
        :return: The number of nodes
        """
        return self.na * self.nl

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
            dist.append(np.linalg.norm(pn - cn))
        # print(sum(dist))
        return sum(dist) / len(previous_nodes)

    def cut_extremities(self, left, right):
        """
        Remove nodes under left value and above right values
        :param left: the distance in meters
        :param right: the distance in meters
        :return: a new list of nodes and new list of edges
        """
        self.update_nodes()
        new_nodes = self.nodes.copy()
        new_edges = self.edges.copy()

        # Left and right values are measured along z axis
        remove_until = 0
        remove_from = 0
        new_nl = self.nl
        for i in range(self.nl):
            # Get first node of this length
            n = self.nodes[i*self.na]
            if n[2] <= left:
                # Remove all the nodes below left
                remove_until += self.na
                new_nl -= 1
            if n[2] >= right:
                # Remove all the nodes above right
                remove_from += self.na
                new_nl -= 1

        # Remove the nodes and edges
        new_nodes = new_nodes[remove_until:-remove_from]
        new_edges = new_edges[remove_until:-remove_from]

        # Update edges' neighbourhood
        for e in new_edges:
            for i in range(4):
                e[i] -= remove_until
                if e[i] < 0:
                    e[i] = -1
        # for e in new_edges[:self.na]:
        #     e[0] = -1
        for e in new_edges[-self.na:]:
            e[2] = -1

        return new_nodes, new_edges, self.na, new_nl

    def downsample(self, factor_angle, factor_length):
        """
        Reduce the number of nodes by a specified factor.
        :param factor_angle: downsampling factor for the number of nodes in the angle
        :param factor_length: downsampling factor for the number of nodes in the length
        :return:
        """

