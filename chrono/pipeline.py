# ------------------------------------------------------------------------------
# Name:        Cloth Simulation
# Purpose: Create a pipeline that:
# # 1. Load and display the target shape (like a rigid cylinder, load an .STL file)
# # 2. Create and display the textile shape that is gonna wrap the target shape
# # 3. Apply the physical simulation of the textile on the target shape
# # 4. Once stabilized, the textile shape become the target shape
# # 5. Apply the force direct algorithm to estimate the target shape with a new textile shape
#
# Author:      Sebastien Richoz
#
# Created:     04/04/2019
# Copyright:   Smartsensotics, Wearable Technologies Lab
# ------------------------------------------------------------------------------

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.mkl as mkl
from math import pi as PI
from sleeve_shellreissner import SleeveShellReissner
import chrono_utils as tool

print("Cloth Simulation: Pipeline for the smartsensotics project")

# Change this path to asset path, if running from other working dir.
# It must point to the data folder, containing GUI assets (textures, fonts, meshes, etc.)
chrono.SetChronoDataPath("../data/")

# Set global variables
NNODES_ANGLE = 16  # number of nodes in the circumference of the sleeve
NNODES_LENGTH = 10  # number of nodes in the length of the sleeve
SHAPE_PATH = 'shapes/cyl30a.obj'

TYPE = 'SMC'  # SMC (Smooth contact, for fea) | NSC (non-smooth contact, for solids)
SOLVER = 'MKL'  # MKL (more precise for FEA elements) | '' (default one)
SET_MATERIAL = False  # If True, will set a skin material property to the target shape (rigid body)

UNIT_FACTOR = 0.01
HUMAN_DENSITY = 98.5  # kg/m^3
# ---------------------------------------------------------------------
#
# Create the simulation system and add items
mysystem = chrono.ChSystemSMC()
contact_method = chrono.ChMaterialSurface.SMC
if TYPE == 'NSC':
    mysystem = chrono.ChSystemNSC()
    contact_method = chrono.ChMaterialSurface.NSC

#mysystem.Set_G_acc(chrono.ChVectorD(0., 0., 0.))

# Set the global collision margins. This is especially important for very large or
# very small objects. Set this before creating shapes. Not before creating mysystem.
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0001)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0001)

# ---------------------------------------------------------------------
#
# Load and display the target shape

# Change the millimeters units into meters
filepath = tool.obj_from_millimeter(chrono.GetChronoDataPath() + SHAPE_PATH, UNIT_FACTOR, "_meters")

shape = chrono.ChBody(contact_method)
shape.SetDensity(HUMAN_DENSITY)
shape.SetBodyFixed(True)
shape_mesh = chrono.ChObjShapeFile()
shape_mesh.SetFilename(filepath)
shape.AddAsset(shape_mesh)
tmc = chrono.ChTriangleMeshConnected()
tmc.LoadWavefrontMesh(filepath)
shape.GetCollisionModel().ClearModel()
shape.GetCollisionModel().AddTriangleMesh(tmc, True, True)
shape.GetCollisionModel().BuildModel()
shape.SetShowCollisionMesh(True)
shape.SetCollide(True)

# Get shape measures
bbmin = chrono.ChVectorD()
bbmax = chrono.ChVectorD()
shape.GetTotalAABB(bbmin, bbmax)
bbmin = eval(str(bbmin))
bbmax = eval(str(bbmax))
shape_length = bbmax[2] - bbmin[2]
shape_diameter = bbmax[0] - bbmin[0]
shape.SetMass(HUMAN_DENSITY*PI*(shape_diameter/2.)**2*shape_length)

# Move shape to the center
shape.SetPos(chrono.ChVectorD(-shape_diameter / 2.,
                              -shape_diameter / 2.,
                              -shape_length / 2.))
shape.SyncCollisionModels()
mysystem.Add(shape)

# Add Skin smooth contact to that shape with some properties (if SET_MATERIAL is set to True)
skin_material = chrono.ChMaterialSurfaceSMC()
skin_material.SetAdhesion(0.01)
skin_material.SetYoungModulus(0.00007)
skin_material.SetPoissonRatio(0.0)
skin_material.SetFriction(0.10)
skin_material.SetRestitution(0.0)
if SET_MATERIAL:
    shape.SetMaterialSurface(skin_material)
# Add a skin texture
skin_texture = chrono.ChTexture()
skin_texture.SetTextureFilename(chrono.GetChronoDataPath() + 'textures/skin.jpg')
shape.GetAssets().push_back(skin_texture)

# ---------------------------------------------------------------------
#
# Create the wrapping sleeve. Must be bigger than the shape
print("Create the wrapping sleeve")

# Create the material property of the mesh
rho = 152.2  # material density
E = 8e4  # Young's modulus 11e6
nu = 0.5  # 0.5  # Poisson ratio
alpha = 1.  # 0.3  # shear factor
beta = 0.2  # torque factor
cloth_material = fea.ChMaterialShellReissnerIsothropic(rho, E, nu,
                                                       alpha, beta)
cloth_length = shape_length
cloth_radius = 0.8 * shape_diameter / 2.0  # TODO make the mesh smaller than the shape
node_mass = 0.01
sleeve_thickness = 0.01
alphadamp = 0.07
sleeve = SleeveShellReissner(cloth_length, cloth_radius, NNODES_ANGLE, NNODES_LENGTH,
                             cloth_material, node_mass, sleeve_thickness, alphadamp,
                             shift_z=-shape_length / 2.
                             )
cloth_mesh = sleeve.get_mesh()

# Add a contact surface mesh
# Add a material surface
contact_material = chrono.ChMaterialSurfaceSMC()
# contact_material.SetYoungModulus(11e5)
contact_material.SetFriction(0.8)
# contact_material.SetRestitution(0.)
# contact_material.SetAdhesion(0.9)
if TYPE == 'NSC':
    contact_material = chrono.ChMaterialSurfaceNSC()

sphere_swept_thickness = 0.008
mcontact = fea.ChContactSurfaceMesh()
cloth_mesh.AddContactSurface(mcontact)
mcontact.AddFacesFromBoundary(sphere_swept_thickness)
mcontact.SetMaterialSurface(contact_material)

# TODO move the extremities of the sleeve to the extremities of the shape

# Fix the extremities of the sleeve to the cylinder
sleeve.fix_extremities(shape, mysystem)

# Extend the sleeve. It will be released after some iterations.
sleeve.extend(10000. * UNIT_FACTOR)

# # ---------------------------------------------------------------------
# # VISUALIZATION
mvisualizeClothcoll = fea.ChVisualizationFEAmesh(cloth_mesh)
mvisualizeClothcoll.SetWireframe(True)
mvisualizeClothcoll.SetFEMglyphType(fea.ChVisualizationFEAmesh.E_GLYPH_NODE_DOT_POS)
mvisualizeClothcoll.SetSymbolsThickness(1.*UNIT_FACTOR)
cloth_mesh.AddAsset(mvisualizeClothcoll)

# Add mesh to the system
#cloth_mesh.SetAutomaticGravity(False)
mysystem.AddMesh(cloth_mesh)
#
# ---------------------------------------------------------------------
# IRRLICHT
# Create an Irrlicht application to visualize the system
#
myapplication = chronoirr.ChIrrApp(mysystem, 'Cloth Simulation', chronoirr.dimension2du(1024, 768))

myapplication.AddTypicalSky()
myapplication.AddTypicalLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
myapplication.AddTypicalCamera(chronoirr.vector3df(shape_length, shape_length/2., shape_length/2.))
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
if SOLVER == 'MKL':
    msolver = mkl.ChSolverMKLcsm()
    mysystem.SetSolver(msolver)
    myapplication.SetTimestep(0.001)

step = 0
threshold = 0.0001
while myapplication.GetDevice().run():
    print('step', step)
    myapplication.BeginScene()
    myapplication.DrawAll()
    myapplication.DoStep()
    myapplication.EndScene()

    if step == 70:
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
