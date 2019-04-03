# ------------------------------------------------------------------------------
# Name:        Inverse modeling of a 3D sleeve wrapping a rigid body
# Purpose:
#
# Author:      Sebastien Richoz
#
# Created:     01/04/2019
# Copyright:   Wearable Technologies Lab
# ------------------------------------------------------------------------------

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.mkl as mkl
from gen_cylinder import gen_cylinder
from shapeopt_force import shapeopt_force
from math import pi as PI
import sleeve_shellreissner

print("Cloth Simulation: Inverse model of a sleeve wrapping a cylinder with direct forces")

# Change this path to asset path, if running from other working dir.
# It must point to the data folder, containing GUI assets (textures, fonts, meshes, etc.)
chrono.SetChronoDataPath("../data/")

CYLINDER_HEIGHT = 2
CYLINDER_RADIUS = 0.25
CLOTH_LENGTH = 1.5 * CYLINDER_HEIGHT
CLOTH_RADIUS = 2.0 * CYLINDER_RADIUS

NA = 20  # Number of nodes in the circumference
NL = 10  # Number of nodes in the length

# ---------------------------------------------------------------------
#
# Create the simulation system and add items
mysystem = chrono.ChSystemSMC()

# Generate the rigid body (target shape)
rigid_nodes, rigid_edges = gen_cylinder(CYLINDER_RADIUS, CYLINDER_HEIGHT, NA, NL,
                                        shift_z=-CYLINDER_HEIGHT / 2)
rigid_mesh = fea.ChMesh()
for rn in rigid_nodes:
    fea_node = fea.ChNodeFEAxyzD(chrono.ChVectorD(rn[0], rn[1], rn[2]))
    rigid_mesh.AddNode(fea_node)

# Generate the shape to optimize (cloth)
cloth_nodes, cloth_edges = gen_cylinder(CLOTH_RADIUS, CLOTH_LENGTH, NA, NL,
                                        shift_z=-CLOTH_LENGTH / 2)

cloth_mesh = fea.ChMesh()
cloth_fea_nodes = []
for cn in cloth_nodes:
    fea_node = fea.ChNodeFEAxyzD(chrono.ChVectorD(cn[0], cn[1], cn[2]))
    fea_node.SetMass(0.001)
    cloth_fea_nodes.append(fea_node)
    cloth_mesh.AddNode(fea_node)

# Create a skin texture
skin_texture = chrono.ChTexture()
skin_texture.SetTextureFilename(chrono.GetChronoDataPath() + 'textures/skin.jpg')
# Create a fixed rigid cylinder
mcylinder = chrono.ChBodyEasyCylinder(CYLINDER_RADIUS, CYLINDER_HEIGHT,   # radius, height
                                      1000,  # density
                                      False, True,
                                      chrono.ChMaterialSurface.SMC)
mcylinder.SetBodyFixed(True)
mcylinder.SetPos(chrono.ChVectorD(0, 0, 0))
qCylinder = chrono.Q_from_AngX(90 * chrono.CH_C_DEG_TO_RAD)
mcylinder.SetRot(qCylinder)
mcylinder.SetMass(1000*PI*CYLINDER_RADIUS**2*CYLINDER_HEIGHT)
mcylinder.SetShowCollisionMesh(False)
color = chrono.ChColorAsset()
color.SetColor(chrono.ChColor(0.4, 0.3, 0.1, 0.5))
mcylinder.AddAsset(color)
#mcylinder.GetAssets().push_back(skin_texture)
mysystem.Add(mcylinder)



# TODO after optimization is working
# Add a contact surface mesh
# Add a material surface

# ---------------------------------------------------------------------
# VISUALIZATION
viz_rigid = fea.ChVisualizationFEAmesh(rigid_mesh)
viz_rigid.SetWireframe(True)
viz_rigid.SetSmoothFaces(False)
viz_rigid.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_SURFACE)
viz_rigid.SetFEMglyphType(fea.ChVisualizationFEAmesh.E_GLYPH_NODE_DOT_POS)
viz_rigid.SetSymbolsThickness(0.03)
rigid_mesh.AddAsset(viz_rigid)

viz_cloth = fea.ChVisualizationFEAmesh(cloth_mesh)
viz_cloth.SetWireframe(True)
viz_cloth.SetFEMglyphType(fea.ChVisualizationFEAmesh.E_GLYPH_NODE_DOT_POS)
viz_cloth.SetSymbolsThickness(0.01)
cloth_mesh.AddAsset(viz_cloth)

# Add mesh to the system
rigid_mesh.SetAutomaticGravity(False)
cloth_mesh.SetAutomaticGravity(False)

mysystem.AddMesh(rigid_mesh)
mysystem.AddMesh(cloth_mesh)

# ---------------------------------------------------------------------
# IRRLICHT
# Create an Irrlicht application to visualize the system
#
myapplication = chronoirr.ChIrrApp(mysystem, 'Cloth Simulation', chronoirr.dimension2du(1024, 768))

myapplication.AddTypicalSky()
myapplication.AddTypicalLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
myapplication.AddTypicalCamera(chronoirr.vector3df(2, 1.5, 1.5))
myapplication.AddTypicalLights()

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

step = 0
current_nodes = cloth_nodes
while myapplication.GetDevice().run():
    myapplication.BeginScene()
    myapplication.DrawAll()
    myapplication.DoStep()

    print('Step', step)
    if step < 100:
        # Apply the force optimization algorithm and visualize each iteration
        updated_nodes, fvall = shapeopt_force(current_nodes, cloth_edges,
                                       rigid_nodes, rigid_edges)

        # Update the force vector of the nodes
        # for fea_n, fv in zip(cloth_fea_nodes, fvall):
        #     # c_fea_n = fea.CastToChNodeFEAxyzDShared(fea_n)
        #     # print('Could be cast to Node?', c_fea_n.IsNull())
        #     fea_n.SetForce(chrono.ChVectorD(fv[0], fv[1], fv[2]))

        # Update the position of the nodes
        for fea_n, un in zip(cloth_fea_nodes, updated_nodes):
            # c_fea_n = fea.CastToChNodeFEAxyzDShared(fea_n)
            # print('Could be cast to Node?', c_fea_n.IsNull())
            cloth_mesh.AddNode(
                fea.ChNodeFEAxyzD(chrono.ChVectorD(un[0], un[1], un[2])))

    current_nodes = updated_nodes

    step += 1
    myapplication.EndScene()
