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
import numpy as np

# SetPos Not working. Doesnt understand inheritance.
#node = fea.ChNodeFEAxyzD(chrono.ChVectorD(0,0,0))
#node.SetPos(chrono.ChVectorD(1,1,1))

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
nA = fea.ChNodeFEAxyz(chrono.ChVectorD(0., 0., 0.))
nB = fea.ChNodeFEAxyz(chrono.ChVectorD(1., 0., 0.))
nC = fea.ChNodeFEAxyz(chrono.ChVectorD(1., 0., 1.))
nD = fea.ChNodeFEAxyz(chrono.ChVectorD(0., 0., 1.))
nE = fea.ChNodeFEAxyz(chrono.ChVectorD(0., 1., 0.))
nF = fea.ChNodeFEAxyz(chrono.ChVectorD(1., 1., 0.))
nG = fea.ChNodeFEAxyz(chrono.ChVectorD(1., 1., 1.))
nH = fea.ChNodeFEAxyz(chrono.ChVectorD(0., 1., 1.))
# nA.SetMass(10.)
# nB.SetMass(10.)
# nC.SetMass(10.)
# nD.SetMass(10.)
# nE.SetMass(10.)
# nF.SetMass(10.)
# nG.SetMass(10.)
# nH.SetMass(10.)
m.AddNode(nA)
m.AddNode(nB)
m.AddNode(nC)
m.AddNode(nD)
#m.AddNode(nE)
#m.AddNode(nF)
#m.AddNode(nG)
#m.AddNode(nH)
#elem = fea.ChElementBrick()
#elem.SetNodes(nA,nB,nC,nD,nE,nF,nG,nH)
#mat = chrono.ChContinuumElastic()
#elem.SetMaterial(mat)
#m.AddElement(elem)
#b1 = fea.ChElementSpring()
#b1.SetNodes(nA, nB)
#b1.SetSpringK(1000)
#b2 = fea.ChElementSpring()
#b2.SetNodes(nB, nC)
#b2.SetSpringK(1000)
#b3 = fea.ChElementSpring()
#b3.SetNodes(nC, nD)
#b3.SetSpringK(1000)
##b4 = fea.ChElementSpring()
#b4.SetNodes(nD, nA)
#b4.SetSpringK(1000)
#b1.SetBarArea(10.0)
#b1.SetBarDensity(1000)
#b2 = fea.ChElementBar()
#b2.SetNodes(nB, nC)
#b2.SetBarArea(0.05)
#b3 = fea.ChElementBar()
#b3.SetNodes(nC, nD)
#b3.SetBarArea(0.05)
#m.AddElement(b1)
#m.AddElement(b2)
#m.AddElement(b3)
#m.AddElement(b4)

nA.SetForce(chrono.ChVectorD(10, 10, 0))

# ---------------------------------------------------------------------
# Test importing STL files
t_mesh = chrono.ChTriangleMeshConnected()
t_mesh.LoadWavefrontMesh(chrono.GetChronoDataPath() + '/shapes/Rod_40x200.stl')

# ---------------------------------------------------------------------

mvisualizeClothBrick = fea.ChVisualizationFEAmesh()
mvisualizeClothBrick.SetWireframe(True)
mvisualizeClothBrick.SetFEMglyphType(fea.ChVisualizationFEAmesh.E_GLYPH_NODE_DOT_POS)
mvisualizeClothBrick.SetSymbolsThickness(0.1)
m.AddAsset(mvisualizeClothBrick)

viz = fea.ChVisualizationFEAmesh(m)
viz.SetWireframe(True)
viz.SetSmoothFaces(True)
viz.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_SURFACE)
m.AddAsset(viz)

# Add mesh to the system
mysystem.AddMesh(m)
m.SetAutomaticGravity(False)

# ---------------------------------------------------------------------
# IRRLICHT
# Create an Irrlicht application to visualize the system
#
myapplication = chronoirr.ChIrrApp(mysystem, 'Cloth Simulation', chronoirr.dimension2du(1024, 768))

myapplication.AddTypicalSky()
myapplication.AddTypicalLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
myapplication.AddTypicalCamera()
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

while myapplication.GetDevice().run():
    myapplication.BeginScene()
    myapplication.DrawAll()
    myapplication.DoStep()
    myapplication.EndScene()
