# ------------------------------------------------------------------------------
# Name:        Cloth Simulation
# Purpose:
#
# Author:      Sebastien Richoz
#
# Created:     20/03/2019
# Copyright:   Wearable Technologies Lab
# ------------------------------------------------------------------------------

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.mkl as mkl
import chrono_utils as tool
import math


def reconstruct_shape(na, nl, unit_factor, ring_gap=0.125,
                      shift_x=0., shift_y=0., shift_z=0.):
    str = """30.85214019
31.00933593
30.37788472
30.04299646
30.13068797
30.35192575
29.92635976
29.48857666"""
    radiuses = [eval(s) * unit_factor for s in str.split('\n')]
    len = ring_gap * nl
    nodes = []
    edges = []
    nidx = 0
    for l in range(nl):
        cl = len / (nl - 1) * l + shift_z  # current length
        for a in range(na):
            ca = a / na * 2. * math.pi  # current angle in radians
            x = radiuses[l] * math.cos(ca) + shift_x
            y = radiuses[l] * math.sin(ca) + shift_y
            v = [x, y, cl]
            nodes.append(v)

            # Make edges
            # [left_node, next_angle_node, right_node, previous_angle_node]
            e = [-1] * 4

            # Link left
            if l > 0:
                e[0] = nidx - na
                # e[0] = na * (l - 1) + j
            # link right
            if l < nl - 1:
                e[2] = nidx + na
                # e[2] = na * (l + 1) + j
            # link to previous angle
            if a > 0:
                e[3] = nidx - 1
                # e[3] = na * l + a - 1
            else:
                e[3] = nidx + (na - 1)
                # e[3] = na * (l + 1) - 1
            # link to next angle
            if a < na - 1:
                e[1] = nidx + 1
                # e[1] = na * l + a + 1
            else:
                e[1] = nidx - (na - 1)

            edges.append(e)
            nidx += 1

    return nodes, edges


# Change this path to asset path, if running from other working dir.
# It must point to the data folder, containing GUI assets (textures, fonts, meshes, etc.)
chrono.SetChronoDataPath("../data/")

UNIT_FACTOR = 0.01
RING_GAP = 12.5 * UNIT_FACTOR
SHAPE_THICKNESS = 0.01
SHAPE_PATH = 'shapes/printed_April18/Cone_32.obj'

metrics = ['mm', 'cm', 'dm', 'm']
metric = metrics[int(math.fabs(round(math.log(UNIT_FACTOR, 10))))]
HUMAN_DENSITY = 198.5  # Dkg/m^3

# ---------------------------------------------------------------------
#
# Create the simulation system and add items
mysystem = chrono.ChSystemSMC()
contact_method = chrono.ChMaterialSurface.SMC
mysystem.Set_G_acc(chrono.ChVectorD(0., 0., 0.))

filepath = tool.obj_from_millimeter(chrono.GetChronoDataPath() + SHAPE_PATH, UNIT_FACTOR, f"_{metric}")

# Import the shape
shape = tool.load_shape(filepath, contact_method, 'textures/skin.jpg')
shape.SetDensity(HUMAN_DENSITY)

# Get shape bounding box dimensions
bbmin, bbmax = chrono.ChVectorD(), chrono.ChVectorD()
shape.GetTotalAABB(bbmin, bbmax)
bbmin, bbmax = eval(str(bbmin)), eval(str(bbmax))
bb_dx = bbmax[0] - bbmin[0]
bb_dy = bbmax[1] - bbmin[1]
bb_dz = bbmax[2] - bbmin[2]
shape.SetMass(100 * HUMAN_DENSITY * bb_dx * bb_dy * bb_dz)

# Align shape to the center of axis system
shape.SetPos(chrono.ChVectorD(-bb_dx / 2. - bbmin[0],
                              -bb_dy / 2. - bbmin[1],
                              -bb_dz / 2. - bbmin[2]))
shape.SyncCollisionModels()
mysystem.Add(shape)

# Reconstruct 3D shape from sensors
na = 8
nl = 8
nodes, edges = reconstruct_shape(na, nl, UNIT_FACTOR, ring_gap=RING_GAP,
                                 shift_z=-RING_GAP * nl / 2.0)

# mesh
mesh = fea.ChMesh()

# nodes
noderot = chrono.ChQuaternionD(chrono.QUNIT)
fea_nodes = []
for n in nodes:
    fn = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(n[0], n[1], n[2]), noderot))
    fn.SetMass(0.1)
    fn.SetFixed(False)
    mesh.AddNode(fn)
    fea_nodes.append(fn)

# material of each element
rho = 152.2  # 152.2 material density
E = 8e20  # Young's modulus 8e4
nu = 0.5  # 0.5  # Poisson ratio
alpha = 1.0  # 0.3  # shear factor
beta = 0.2  # torque factor
material = fea.ChMaterialShellReissnerIsothropic(rho, E, nu, alpha, beta)
alphadamp = 0.1
# elements
elements = []
for i in range(nl - 1):
    for j in range(na):

        # Make elements
        elem = fea.ChElementShellReissner4()

        if j == na - 1:
            # Connect last nodes to the first ones
            elem.SetNodes(
                fea_nodes[(i + 1) * na],  # top right
                fea_nodes[i * na],  # top left
                fea_nodes[j + i * na],  # bottom left
                fea_nodes[j + (i + 1) * na]  # bottom right
            )
        else:
            elem.SetNodes(
                fea_nodes[j + 1 + (i + 1) * na],  # top right
                fea_nodes[j + 1 + i * na],  # top left
                fea_nodes[j + i * na],  # bottom left
                fea_nodes[j + (i + 1) * na]  # bottom right
            )

        elem.AddLayer(SHAPE_THICKNESS, 0 * chrono.CH_C_DEG_TO_RAD, material)
        elem.SetAlphaDamp(alphadamp)
        elem.SetAsNeutral()

        mesh.AddElement(elem)
        elements.append(elem)

viz = fea.ChVisualizationFEAmesh(mesh)
viz.SetWireframe(True)
viz.SetFEMglyphType(fea.ChVisualizationFEAmesh.E_GLYPH_NODE_DOT_POS)
viz.SetShellResolution(2)
viz.SetSymbolsThickness(0.02)
mesh.AddAsset(viz)

mvisualizebeamA = fea.ChVisualizationFEAmesh(mesh)
mvisualizebeamA.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_ELEM_BEAM_MZ)
mvisualizebeamA.SetColorscaleMinMax(-0.4, 0.4)
mvisualizebeamA.SetSmoothFaces(True)
mvisualizebeamA.SetWireframe(False)
mesh.AddAsset(mvisualizebeamA)

mysystem.AddMesh(mesh)

# ---------------------------------------------------------------------
# IRRLICHT
# Create an Irrlicht application to visualize the system
#
myapplication = chronoirr.ChIrrApp(mysystem, 'Cloth Simulation', chronoirr.dimension2du(1024, 768))

myapplication.AddTypicalSky()
# myapplication.AddTypicalLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
myapplication.AddTypicalCamera(chronoirr.vector3df(0.3, 0., 0.3))
myapplication.AddTypicalLights()
myapplication.SetShowInfos(False)
# myapplication.SetContactsDrawMode(chronoirr.IrrlichtDevice.)

# ==IMPORTANT!== Use this function for adding a ChIrrNodeAsset to all items
# in the system. These ChIrrNodeAsset assets are 'proxies' to the Irrlicht meshes.
# If you need a finer control on which item really needs a visualization proxy in
# Irrlicht, just use application.AssetBind(myitem) on a per-item basis.
myapplication.AssetBindAll()

# ==IMPORTANT!== Use this function for 'converting' into Irrlicht meshes the assets
# that you added to the bodies into 3D shapes, they can be visualized by Irrlicht!
myapplication.AssetUpdateAll()
mysystem.SetupInitial()

# ---------------------------------------------------------------------
# SIMULATION
# Run the simulation
#
# Change the solver form the default SOR to the MKL Pardiso, more precise for fea.
msolver = mkl.ChSolverMKLcsm()
mysystem.SetSolver(msolver)
myapplication.SetTimestep(0.001)

while myapplication.GetDevice().run():
    myapplication.BeginScene()
    myapplication.DrawAll()
    myapplication.DoStep()
    myapplication.EndScene()
    # chronoirr.ChIrrTools.drawSegment(myapplication.GetVideoDriver(),
    #                                 chrono.ChVectorD(0., 0., 0.),
    #                                 chrono.ChVectorD(.1, .1, .1))
