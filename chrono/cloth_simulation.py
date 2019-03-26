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
from sleeve import Sleeve

print("Cloth Simulation: create and visualize an elastic sleeve around a bumped cylinder")

# Change this path to asset path, if running from other working dir.
# It must point to the data folder, containing GUI assets (textures, fonts, meshes, etc.)
chrono.SetChronoDataPath("../data/")

CYLINDER_HEIGHT = 0.2
CYLINDER_RADIUS = 0.025

CLOTH_LENGTH = 2 * 3.1415 * CYLINDER_RADIUS
CLOTH_RADIUS = 2.0 * CYLINDER_RADIUS
HUMAN_DENSITY = 985  # kg/m^3
EARTH_DENSITY = 5515.3
ALU_DENSITY = 2810

# ---------------------------------------------------------------------
#
# Create the simulation system and add items
mysystem = chrono.ChSystemNSC()

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
skin_material.SetAdhesion(0.0)
skin_material.SetYoungModulus(0.0)
skin_material.SetPoissonRatio(0.0)
skin_material.SetFriction(0.0)
skin_material.SetRestitution(0.0)
# Create a skin texture
skin_texture = chrono.ChTexture()
skin_texture.SetTextureFilename(chrono.GetChronoDataPath() + 'textures/skin.jpg')

# Aluminium non-smooth contact
alu_material = chrono.ChMaterialSurfaceNSC()
alu_material.SetFriction(0.02)
alu_material.SetDampingF(0.0)
alu_material.SetCompliance(0.0000)
alu_material.SetComplianceT(0.00000)
alu_material.SetCohesion(0.0)
alu_material.SetRollingFriction(0.0)

# Create a floor
mfloor = chrono.ChBodyEasyBox(0.5, 0.05, 0.5, EARTH_DENSITY, True, True)
mfloor.SetBodyFixed(True)
#mfloor.SetMaterialSurface(alu_material)

# Create a fixed rigid cylinder
mcylinder = chrono.ChBodyEasyCylinder(CYLINDER_RADIUS, CYLINDER_HEIGHT,   # radius, height
                                      HUMAN_DENSITY,  # density
                                      True, True)
mcylinder.SetBodyFixed(True)
mcylinder.SetPos(chrono.ChVectorD(0, 0.1, 0))
qCylinder = chrono.Q_from_AngX(90 * chrono.CH_C_DEG_TO_RAD)
mcylinder.SetRot(qCylinder)
mcylinder.SetMass(HUMAN_DENSITY*PI*CYLINDER_RADIUS**2*CYLINDER_HEIGHT)
mcylinder.GetAssets().push_back(skin_texture)
#mcylinder.SetMaterialSurface(alu_material)

# Add a bump to the cylinder
sphere_radius = CYLINDER_RADIUS / 1.1
msphere = chrono.ChBodyEasySphere(sphere_radius, HUMAN_DENSITY, True, True)  # radius, density
msphere.SetBodyFixed(True)
msphere.SetPos(chrono.ChVectorD(0, 0.1+CYLINDER_RADIUS*0.4, 0))
msphere.SetMass(HUMAN_DENSITY*(4/3)*PI*sphere_radius**3)
msphere.GetAssets().push_back(skin_texture)
#msphere.SetMaterialSurface(alu_material)

# Create a falling sphere to test collision
sphere2_radius = 0.01
falling_sphere = chrono.ChBodyEasySphere(sphere2_radius, ALU_DENSITY, True, True)
falling_sphere.SetBodyFixed(False)
falling_sphere.SetPos(chrono.ChVectorD(0.01, 0.2, 0.0))
falling_sphere.SetMass(ALU_DENSITY * (4 / 3) * PI * sphere2_radius ** 3)
#falling_sphere.SetMaterialSurface(alu_material)

mat = falling_sphere.GetMaterialSurfaceBase()

mysystem.Add(mfloor)
mysystem.Add(mcylinder)
mysystem.Add(msphere)
mysystem.Add(falling_sphere)

# ----------------------------------------------
# Create a drape wrapping the cylinder

# Create a 3D thin mesh
nnodes_angle = 16  # min 2
nnodes_length = 4  # min 2

# Create the material property of the mesh
rho = 0.0
E = 0.01e9  # rubber
nu = 0.0
shear_factor = 0.1
torque_factor = 0.1
cloth_material = fea.ChMaterialShellReissnerIsothropic(rho, E, nu,
                                                       shear_factor, torque_factor)

sleeve = Sleeve(CLOTH_LENGTH, CLOTH_RADIUS, nnodes_angle, nnodes_length,
                cloth_material, shift_y=0.1, shift_z=-CLOTH_LENGTH/2)
cloth_mesh = sleeve.getMesh()

# Add a contact surface mesh
# Add a material surface
contact_material = chrono.ChMaterialSurfaceNSC()
# contact_material.SetYoungModulus(6e4)
# contact_material.SetFriction(0.3)
# contact_material.SetRestitution(0.2)
# contact_material.SetAdhesion(0.0)

# TODO wait for answer http://projectchrono.org/forum/?place=msg%2Fprojectchrono%2F5b-GatNDPyY%2FMQFKz5yHCQAJ
sphere_swept_thickness = 0.001
mcontact = fea.ChContactSurfaceMesh()
cloth_mesh.AddContactSurface(mcontact)
#mcontact.AddFacesFromBoundary(sphere_swept_thickness)
#mcontact.SetMaterialSurface(contact_material)

# ==Asset== attach a visualization of the FEM mesh.
mvisualizeCloth = fea.ChVisualizationFEAmesh(cloth_mesh)
mvisualizeCloth.SetSmoothFaces(False)
mvisualizeCloth.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_SURFACE)
mvisualizeCloth.SetFEMglyphType(fea.ChVisualizationFEAmesh.E_GLYPH_NODE_DOT_POS)
mvisualizeCloth.SetColorscaleMinMax(0.0, 5.0)
cloth_mesh.AddAsset(mvisualizeCloth)

mvisualizeClothcoll = fea.ChVisualizationFEAmesh(cloth_mesh)
mvisualizeClothcoll.SetWireframe(True)
mvisualizeClothcoll.SetFEMglyphType(fea.ChVisualizationFEAmesh.E_GLYPH_NODE_CSYS)
mvisualizeClothcoll.SetSymbolsThickness(0.007)
cloth_mesh.AddAsset(mvisualizeClothcoll)

# Add mesh to the system
cloth_mesh.SetAutomaticGravity(True)
print('GRAVITY', cloth_mesh.GetAutomaticGravity())
mysystem.AddMesh(cloth_mesh)


# ---------------------------------------------------------------------
#
# Create an Irrlicht application to visualize the system
#
myapplication = chronoirr.ChIrrApp(mysystem, 'Cloth Simulation', chronoirr.dimension2du(1024, 768))

myapplication.AddTypicalSky()
myapplication.AddTypicalLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
myapplication.AddTypicalCamera(chronoirr.vector3df(0.2, 0.2, 0.3))
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
#
# Run the simulation
#

# Change the solver form the default SOR to the MKL Pardiso, more precise for fea.
#msolver = mkl.ChSolverMKLcsm()
#mysystem.SetSolver(msolver)

myapplication.SetTimestep(0.003)

while myapplication.GetDevice().run():
    myapplication.BeginScene()
    myapplication.DrawAll()
    myapplication.DoStep()
    myapplication.EndScene()
