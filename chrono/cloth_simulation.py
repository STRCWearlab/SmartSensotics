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
from math import pi as PI
from sleeve_shellreissner import SleeveShellReissner
from sleeve_brick import SleeveBrick

print("Cloth Simulation: create and visualize an elastic sleeve around a bumped cylinder")

# Change this path to asset path, if running from other working dir.
# It must point to the data folder, containing GUI assets (textures, fonts, meshes, etc.)
chrono.SetChronoDataPath("../data/")

CYLINDER_HEIGHT = 2
CYLINDER_RADIUS = 0.25

CLOTH_LENGTH = 2 * PI * CYLINDER_RADIUS
CLOTH_RADIUS = 1.0 * CYLINDER_RADIUS
HUMAN_DENSITY = 985  # kg/m^3
EARTH_DENSITY = 5515.3
ALU_DENSITY = 2810

NSC = False  # NSC = True | SMC = False
SOLVER_SOR = False  # Solver MKL is more precise for FEA elements. Set False if you want it.
SET_MATERIAL = False  # True or False to set material property to the rigid bodies

# ---------------------------------------------------------------------
#
# Create the simulation system and add items
mysystem = chrono.ChSystemNSC() if NSC else chrono.ChSystemSMC()

# Set the global collision margins. This is especially important for very large or
# very small objects. Set this before creating shapes. Not before creating mysystem.
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.000)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0001)

# ---------------------------------------------------------------------
#
# Create the simulation system and add items
#

# Skin smooth contact
skin_material = chrono.ChMaterialSurfaceSMC()
skin_material.SetAdhesion(0.01)
skin_material.SetYoungModulus(0.00007)
skin_material.SetPoissonRatio(0.0)
skin_material.SetFriction(0.10)
skin_material.SetRestitution(0.0)
# Create a skin texture
skin_texture = chrono.ChTexture()
skin_texture.SetTextureFilename(chrono.GetChronoDataPath() + 'textures/skin.jpg')

# Aluminium non-smooth contact
alu_material = chrono.ChMaterialSurfaceNSC()
alu_material.SetFriction(0.02)
alu_material.SetDampingF(0.10)
alu_material.SetCompliance(0.00001)
alu_material.SetComplianceT(0.000001)
alu_material.SetCohesion(0.10)
alu_material.SetRollingFriction(0.0)

# Create a fixed rigid cylinder
mcylinder = chrono.ChBodyEasyCylinder(CYLINDER_RADIUS, CYLINDER_HEIGHT,   # radius, height
                                      HUMAN_DENSITY,  # density
                                      True, True,
                                      chrono.ChMaterialSurface.NSC if NSC else chrono.ChMaterialSurface.SMC)
mcylinder.SetBodyFixed(True)
mcylinder.SetPos(chrono.ChVectorD(0, 0, 0))
qCylinder = chrono.Q_from_AngX(90 * chrono.CH_C_DEG_TO_RAD)
mcylinder.SetRot(qCylinder)
mcylinder.SetMass(HUMAN_DENSITY*PI*CYLINDER_RADIUS**2*CYLINDER_HEIGHT)
mcylinder.GetAssets().push_back(skin_texture)
if SET_MATERIAL:
    mcylinder.SetMaterialSurface(alu_material if NSC else skin_material)

# Add a bump to the cylinder
sphere_radius = CYLINDER_RADIUS / 1.1
msphere = chrono.ChBodyEasySphere(sphere_radius, HUMAN_DENSITY, True, True,
                                  chrono.ChMaterialSurface.NSC if NSC else chrono.ChMaterialSurface.SMC)
msphere.SetBodyFixed(True)
msphere.SetPos(chrono.ChVectorD(0, CYLINDER_RADIUS*0.4, 0))
msphere.SetMass(HUMAN_DENSITY*(4/3)*PI*sphere_radius**3)
msphere.GetAssets().push_back(skin_texture)
if SET_MATERIAL:
    msphere.SetMaterialSurface(alu_material if NSC else skin_material)

mysystem.Add(mcylinder)
mysystem.Add(msphere)

# ----------------------------------------------
# Create a drape wrapping the cylinder

# Create a 3D thin mesh
nnodes_angle = 20  # min 2
nnodes_length = 20  # min 2

# Create the material property of the mesh
rho = 1522  # material density
E = 8e3  # Young's modulus 11e6
nu = 0.5  # 0.5  # Poisson ratio
alpha = 10.0  # 0.3  # shear factor
beta = 0.2  # torque factor
cloth_material = fea.ChMaterialShellReissnerIsothropic(rho, E, nu,
                                                       alpha, beta)
node_mass = 10.
sleeve_thickness = 0.1
alphadamp = 0.1
sleeve = SleeveShellReissner(CLOTH_LENGTH, CLOTH_RADIUS, nnodes_angle, nnodes_length,
                             node_mass, sleeve_thickness, alphadamp,
                             cloth_material, shift_y=0, shift_z=-CLOTH_LENGTH/2)

mat_elastic = chrono.ChContinuumElastic(E, nu, rho)
#sleeve_brick = SleeveBrick(CLOTH_LENGTH, 2.0*CLOTH_RADIUS, 0.05, 7, 3,
#                           material=mat_elastic, shift_y=0, shift_z=-CLOTH_LENGTH/2)
cloth_mesh = sleeve.get_mesh()
#cloth_mesh_brick = sleeve_brick.get_mesh()

# Add a contact surface mesh
# Add a material surface
contact_material = chrono.ChMaterialSurfaceSMC()
#contact_material.SetYoungModulus(11e5)
contact_material.SetFriction(0.8)
#contact_material.SetRestitution(0.)
#contact_material.SetAdhesion(0.9)
if NSC:
    contact_material = chrono.ChMaterialSurfaceNSC()

sphere_swept_thickness = 0.008
mcontact = fea.ChContactSurfaceMesh()
cloth_mesh.AddContactSurface(mcontact)
mcontact.AddFacesFromBoundary(sphere_swept_thickness)
mcontact.SetMaterialSurface(contact_material)

# Fix the extremities of the sleeve to the cylinder
sleeve.fix_extremities(mcylinder, mysystem)

# Extend the sleeve. It will be released after some iterations.
sleeve.expand(10.0)

# ---------------------------------------------------------------------
# VISUALIZATION
# ==Asset== attach a visualization of the FEM mesh.
#mvisualizeCloth = fea.ChVisualizationFEAmesh(cloth_mesh)
#mvisualizeCloth.SetSmoothFaces(False)
#mvisualizeCloth.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_SURFACE)
#mvisualizeCloth.SetFEMglyphType(fea.ChVisualizationFEAmesh.E_GLYPH_NODE_DOT_POS)
#mvisualizeCloth.SetColorscaleMinMax(-5.0, 5.0)
#cloth_mesh.AddAsset(mvisualizeCloth)

mvisualizeClothcoll = fea.ChVisualizationFEAmesh(cloth_mesh)
mvisualizeClothcoll.SetWireframe(True)
mvisualizeClothcoll.SetFEMglyphType(fea.ChVisualizationFEAmesh.E_GLYPH_NODE_DOT_POS)
mvisualizeClothcoll.SetSymbolsThickness(0.02)
cloth_mesh.AddAsset(mvisualizeClothcoll)

# mvisualizeClothBrick = fea.ChVisualizationFEAmesh(cloth_mesh_brick)
# mvisualizeClothBrick.SetWireframe(True)
# mvisualizeClothBrick.SetFEMglyphType(fea.ChVisualizationFEAmesh.E_GLYPH_NODE_DOT_POS)
# cloth_mesh_brick.AddAsset(mvisualizeClothBrick)

# Add mesh to the system
cloth_mesh.SetAutomaticGravity(False)
mysystem.AddMesh(cloth_mesh)
#mysystem.AddMesh(m)


# ---------------------------------------------------------------------
# IRRLICHT
# Create an Irrlicht application to visualize the system
#
myapplication = chronoirr.ChIrrApp(mysystem, 'Cloth Simulation', chronoirr.dimension2du(1024, 768))

myapplication.AddTypicalSky()
myapplication.AddTypicalLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
myapplication.AddTypicalCamera(chronoirr.vector3df(1.7, 0.5, 1.5))
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
myapplication.SetTimestep(0.001)
# Change the solver form the default SOR to the MKL Pardiso, more precise for fea.
if SOLVER_SOR is False:
    msolver = mkl.ChSolverMKLcsm()
    mysystem.SetSolver(msolver)
    myapplication.SetTimestep(0.01)

step = 0
threshold = 0.0001
while myapplication.GetDevice().run():
    #print('step', step)

    myapplication.BeginScene()
    myapplication.DrawAll()
    myapplication.DoStep()
    myapplication.EndScene()


    if step == 51:
        sleeve.release()

    # Detect stabilisation of the sleeve
    # When the sleeve movements aren't significant, we extract the position of the nodes
    if step % 5 == 1:
        previous_nodes = sleeve.nodes.copy()
        sleeve.update_nodes()
        mean_sd = sleeve.get_sd_nodes(previous_nodes)
        print(mean_sd)
        if mean_sd < threshold:
            # Set reference position of nodes as current position, for all nodes
            sleeve.update_nodes()
            target_nodes = sleeve.nodes



    step += 1

print('target nodes')
print(target_nodes)
