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
from gen_3DMesh import gen_3d_cylinder_mesh

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

chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0001)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0001)

# ---------------------------------------------------------------------
#
# Create the simulation system and add items
#

# Skin smooth contact
skin_material = chrono.ChMaterialSurfaceSMC()
skin_material.SetAdhesion(1.0)
skin_material.SetYoungModulus(10.0)
skin_material.SetPoissonRatio(0.27)
skin_material.SetFriction(10.0)
skin_material.SetRestitution(10.0)
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

mysystem.Add(mfloor)

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
mcylinder.SetMaterialSurface(alu_material)

# Add a bump to the cylinder
sphere_radius = CYLINDER_RADIUS / 1.1
msphere = chrono.ChBodyEasySphere(sphere_radius, HUMAN_DENSITY, True, True)  # radius, density
msphere.SetBodyFixed(True)
msphere.SetPos(chrono.ChVectorD(0, 0.1+CYLINDER_RADIUS*0.4, 0))
msphere.SetMass(HUMAN_DENSITY*(4/3)*PI*sphere_radius**3)
msphere.GetAssets().push_back(skin_texture)
msphere.SetMaterialSurface(alu_material)

# Test with a falling sphere
sphere2_radius = 0.01
msphere2 = chrono.ChBodyEasySphere(sphere2_radius, ALU_DENSITY, True, True)
msphere2.SetBodyFixed(False)
msphere2.SetPos(chrono.ChVectorD(0.01, 0.2, 0.0))
msphere2.SetMass(ALU_DENSITY*(4/3)*PI*sphere2_radius**3)
msphere2.SetMaterialSurface(alu_material)

mysystem.Add(mcylinder)
mysystem.Add(msphere)
mysystem.Add(msphere2)

# ----------------------------------------------
# Create a drape wrapping the cylinder



# Create a 3D thin mesh
nnodes_angle = 8  # min 2
nnodes_length = 4  # min 2

# Create a mesh, that is a container for groups
# of elements and their referenced nodes.
cloth_mesh = fea.ChMesh()
mysystem.Set_G_acc(chrono.ChVectorD(0., -9.81, 0.))
cloth_mesh.SetAutomaticGravity(True)

cloth_thickness = 0.1
cloth_ri = 0.06
cloth_ro = 0.1

# Create a material
rho = 0.0
E = 0.01e9
nu = 0.0
shear_factor = 0.1
torque_factor = 0.1
cloth_material = fea.ChMaterialShellReissnerIsothropic(rho, E, nu, shear_factor, torque_factor)

cloth_mesh = gen_3d_cylinder_mesh(CLOTH_LENGTH, CLOTH_RADIUS, nnodes_angle, nnodes_length,
                                  cloth_material, shift_y=0.1, shift_z=-CLOTH_LENGTH/2)


# Add a contact surface mesh
contact_material = chrono.ChMaterialSurfaceSMC()
contact_material.SetYoungModulus(6e4)
contact_material.SetFriction(0.2)
contact_material.SetRestitution(0.2)
contact_material.SetAdhesion(0.1)

# TODO wait for answer http://projectchrono.org/forum/?place=msg%2Fprojectchrono%2F5b-GatNDPyY%2FMQFKz5yHCQAJ
#mcontact = fea.ChContactSurfaceMesh()
#cloth_mesh.AddContactSurface(mcontact)
#mcontact.AddFacesFromBoundary(cloth_thickness)
#mcontact.SetMaterialSurface(contact_material)

# ==Asset== attach a visualization of the FEM mesh.
# This will automatically update a triangle mesh (a ChTriangleMeshShape
# asset that is internally managed) by setting  proper
# coordinates and vertex colors as in the FEM elements.
mvisualizeCloth = fea.ChVisualizationFEAmesh(cloth_mesh)
mvisualizeCloth.SetSmoothFaces(False)
mvisualizeCloth.SetWireframe(True)
mvisualizeCloth.SetFEMglyphType(fea.ChVisualizationFEAmesh.E_PLOT_CONTACTSURFACES)
mvisualizeCloth.SetFEMglyphType(fea.ChVisualizationFEAmesh.E_GLYPH_NODE_CSYS)
mvisualizeCloth.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_ELEM_BEAM_MZ)
# mvisualizeCloth.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_CONTACTSURFACES)
# mvisualizeCloth.SetShellResolution(2)
mvisualizeCloth.SetColorscaleMinMax(-0.4, 0.4)
cloth_mesh.AddAsset(mvisualizeCloth)

mysystem.AddMesh(cloth_mesh)


# ---------------------------------------------------------------------
#
# Create an Irrlicht application to visualize the system
#
myapplication = chronoirr.ChIrrApp(mysystem, 'Cloth Simulation', chronoirr.dimension2du(1024, 768))

myapplication.AddTypicalSky()
myapplication.AddTypicalLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
myapplication.AddTypicalCamera(chronoirr.vector3df(0.2, 0.2, 0.3))
#myapplication.AddTypicalCamera(chronoirr.vector3df(0.4, 0, 0))
myapplication.AddTypicalLights()

# ==IMPORTANT!== Use this function for adding a ChIrrNodeAsset to all items
# in the system. These ChIrrNodeAsset assets are 'proxies' to the Irrlicht meshes.
# If you need a finer control on which item really needs a visualization proxy in
# Irrlicht, just use application.AssetBind(myitem) on a per-item basis.
myapplication.AssetBindAll()

# ==IMPORTANT!== Use this function for 'converting' into Irrlicht meshes the assets
# that you added to the bodies into 3D shapes, they can be visualized by Irrlicht!
myapplication.AssetUpdateAll()
# myapplication.AddShadowAll()
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
