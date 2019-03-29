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

# ---------------------------------------------------------------------
#
# Create the simulation system and add items
mysystem = chrono.ChSystemSMC()

# Set the global collision margins. This is especially important for very large or
# very small objects. Set this before creating shapes. Not before creating mysystem.
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.000)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0001)

# ---------------------------------------------------------------------
# Test brick mesh
m = fea.ChMesh()
nA = fea.ChNodeFEAxyz(chrono.ChVectorD(0,0,0))
nB = fea.ChNodeFEAxyz(chrono.ChVectorD(1,0,0))
nC = fea.ChNodeFEAxyz(chrono.ChVectorD(1,0,1))
nD = fea.ChNodeFEAxyz(chrono.ChVectorD(0,0,1))
nE = fea.ChNodeFEAxyz(chrono.ChVectorD(0,1,0))
nF = fea.ChNodeFEAxyz(chrono.ChVectorD(1,1,0))
nG = fea.ChNodeFEAxyz(chrono.ChVectorD(1,1,1))
nH = fea.ChNodeFEAxyz(chrono.ChVectorD(0,1,1))
m.AddNode(nA)
m.AddNode(nB)
m.AddNode(nC)
m.AddNode(nD)
m.AddNode(nE)
m.AddNode(nF)
m.AddNode(nG)
m.AddNode(nH)
elem = fea.ChElementBrick()
elem.SetNodes(nA,nB,nC,nD,nE,nF,nG,nH)
#mat = chrono.ChContinuumElastic()
#elem.SetMaterial(mat)
m.AddElement(elem)


# ---------------------------------------------------------------------

mvisualizeClothBrick = fea.ChVisualizationFEAmesh(m)
mvisualizeClothBrick.SetWireframe(True)
mvisualizeClothBrick.SetFEMglyphType(fea.ChVisualizationFEAmesh.E_GLYPH_NODE_DOT_POS)
#m.AddAsset(mvisualizeClothBrick)

# Add mesh to the system
mysystem.AddMesh(m)


# ---------------------------------------------------------------------
# IRRLICHT
# Create an Irrlicht application to visualize the system
#
myapplication = chronoirr.ChIrrApp(mysystem, 'Cloth Simulation', chronoirr.dimension2du(1024, 768))

myapplication.AddTypicalSky()
myapplication.AddTypicalLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
myapplication.AddTypicalCamera(chronoirr.vector3df(2,2,2))
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
msolver = mkl.ChSolverMKLcsm()
mysystem.SetSolver(msolver)
myapplication.SetTimestep(0.01)

step = 0
while myapplication.GetDevice().run():
    myapplication.BeginScene()
    myapplication.DrawAll()
    myapplication.DoStep()
    step += 1
    print('step', step)

    myapplication.EndScene()
