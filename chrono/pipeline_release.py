# ------------------------------------------------------------------------------
# Name:        Cloth Simulation
# Purpose: Create a pipeline that:
# # 1. Load and display the shape (mesh defined as .obj file)
# # 2. Create the textile sleeve draping the shape
# # 3. Physical simulation of the sleeve draping the cone +
# #    downsampling to match the number of sensors --> save picture
# # 4. shape optimisation simulation --> save picture
#
# Author:      Sebastien Richoz
#
# Created:     04/04/2019
# Copyright:   Smartsensotics, Wearable Technologies Lab
# ------------------------------------------------------------------------------

import numpy as np
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.mkl as mkl
from sleeve_shellreissner import SleeveShellReissner
import chrono_utils as tool
from gen_cylinder import gen_cylinder, downsample
from shapeopt_force import shapeopt_force
import math
import geo

print("Cloth Simulation: Physical (pink) and optimisation (green) simulations")

# Change this path to asset path, if running from other working dir.
# It must point to the data folder, containing GUI assets (textures, fonts, meshes, etc.)
chrono.SetChronoDataPath("../data/")

# Global -- Changeable setup
NEIGHBOURS = 4
SAVE_VIDEO = False
# Shape
SHAPE_PATH = 'shapes/printed_April18/Cone_32.obj'
# Sleeve
NSENSORS_ANGLE = 8
NSENSORS_LENGTH = 8
NNODES_ANGLE = NSENSORS_ANGLE * 2  # Must be a multiple of NSENSOR
NNODES_LENGTH = 2 + NSENSORS_LENGTH * 2  # Must be a multiple of NSENSOR

# Global -- Better not change
SET_MATERIAL = False  # If True, will set a skin material property to the shape (rigid body)
UNIT_FACTOR = 0.01
factor_min_radius = 0.7
metrics = ['mm', 'cm', 'dm', 'm']
metric = metrics[int(math.fabs(round(math.log(UNIT_FACTOR, 10))))]
filename = SHAPE_PATH.split('/')[-1].split('.')[0]
HUMAN_DENSITY = 198.5  # Dkg/m^3

# ---------------------------------------------------------------------
# CHRONO SETUP
# Create the simulation system and add items
mysystem = chrono.ChSystemSMC()
mysystem.Set_G_acc(chrono.ChVectorD(0., 0., 0.))  # Remove gravity
contact_method = chrono.ChMaterialSurface.SMC

# Set global collision margins
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0001)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0001)

# ---------------------------------------------------------------------
# SHAPE
# Load and display the shape

# Change the millimeters units into meters
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
min_radius = tool.get_shape_min_radius(SHAPE_PATH, bb_dx, bb_dy) * UNIT_FACTOR
offset = 0.02 * bb_dz
shape.SetMass(100 * HUMAN_DENSITY * bb_dx * bb_dy * bb_dz)

# Align shape to the center of axis system
shape.SetPos(chrono.ChVectorD(-bb_dx / 2. - bbmin[0],
                              -bb_dy / 2. - bbmin[1],
                              -bb_dz / 2. - bbmin[2]))
shape.SyncCollisionModels()
mysystem.Add(shape)

# ---------------------------------------------------------------------
# DISKS TO FIX THE SLEEVE
# Add fixed extremities to the shape in order to fix the future mesh.
left_cyl, right_cyl = tool.build_external_cylinder(factor_min_radius * min_radius, bb_dz,
                                                   HUMAN_DENSITY, contact_method, offset)
mysystem.Add(left_cyl)
mysystem.Add(right_cyl)

# ---------------------------------------------------------------------
# SLEEVE
# Create the wrapping sleeve.

# Create the material property of the mesh
rho = 152.2  # 152.2 material density
E = 8e4  # Young's modulus 8e4
nu = 0.5  # 0.5  # Poisson ratio
alpha = 1.0  # 0.3  # shear factor
beta = 0.2  # torque factor
cloth_material = fea.ChMaterialShellReissnerIsothropic(rho, E, nu, alpha, beta)

# Create the mesh. At rest, the radius of the mesh is smaller than the min_radius of the shape
cloth_length = bb_dz + 2 * offset
cloth_radius = factor_min_radius * min_radius
node_mass = 0.1
sleeve_thickness = 0.015
alphadamp = 0.05
sleeve = SleeveShellReissner(cloth_length, cloth_radius, NNODES_ANGLE, NNODES_LENGTH, NEIGHBOURS,
                             cloth_material, node_mass, sleeve_thickness, alphadamp,
                             shift_z=-bb_dz / 2. - offset)

# Set the rest position as the actual position
sleeve.relax()
cloth_mesh = sleeve.get_mesh()

# Add a contact surface mesh with material properties
contact_material = chrono.ChMaterialSurfaceSMC()
# contact_material.SetFriction(0.1)
# contact_material.SetAdhesion(0.5)
contact_material.SetYoungModulus(30e5)
sphere_swept_thickness = 0.008
mcontact = fea.ChContactSurfaceMesh()
cloth_mesh.AddContactSurface(mcontact)
mcontact.AddFacesFromBoundary(sphere_swept_thickness)
mcontact.SetMaterialSurface(contact_material)

# Fix the extremities of the sleeve to the disks
sleeve.fix_extremities(left_cyl, right_cyl, mysystem)

# Extend the sleeve. It will be released after some iterations.
# TODO check for a generic way of computing the expanding force
sleeve.expand(1 / (NNODES_ANGLE * NNODES_LENGTH) * 7000000. * UNIT_FACTOR * bb_dx)

# ---------------------------------------------------------------------
# OPTIMISATION SETUP
# Prepare the visualization of the optimisation algorithm

# Mesh computed by the physical simulation
target_mesh = fea.ChMesh()
# The mesh that will approximate the target mesh
inf_mesh = fea.ChMesh()

# # ---------------------------------------------------------------------
# # VISUALIZATION
mvisualizeClothcoll = fea.ChVisualizationFEAmesh(cloth_mesh)
mvisualizeClothcoll.SetWireframe(True)
mvisualizeClothcoll.SetFEMglyphType(fea.ChVisualizationFEAmesh.E_GLYPH_NODE_DOT_POS)
mvisualizeClothcoll.SetSymbolsThickness(1. * UNIT_FACTOR)
#cloth_mesh.AddAsset(mvisualizeClothcoll)

viz_cloth = fea.ChVisualizationFEAmesh(inf_mesh)
viz_cloth.SetWireframe(True)
viz_cloth.SetFEMglyphType(fea.ChVisualizationFEAmesh.E_GLYPH_NODE_DOT_POS)
viz_cloth.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_NONE)
viz_cloth.SetSymbolsThickness(0.005)
# viz_cloth.SetDefaultSymbolsColor(chrono.ChColor(0.2,0.3,0.2))  # TODO bug lib
# viz_cloth.SetDefaultMeshColor(chrono.ChColor(0.2,0.3,0.2))  # TODO bug lib
inf_mesh.AddAsset(viz_cloth)

viz_rigid_mesh = fea.ChVisualizationFEAmesh(target_mesh)
viz_rigid_mesh.SetFEMglyphType(fea.ChVisualizationFEAmesh.E_GLYPH_NODE_DOT_POS)
viz_rigid_mesh.SetSymbolsThickness(2. * UNIT_FACTOR)
# viz_rigid_mesh.SetDefaultSymbolsColor(chrono.ChColor(0.2, 0.2, 0.2))  # TODO bug lib
target_mesh.AddAsset(viz_rigid_mesh)

viz_rigid_mesh_beam = fea.ChVisualizationFEAmesh(target_mesh)
viz_rigid_mesh_beam.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_ELEM_BEAM_MZ)
viz_rigid_mesh_beam.SetColorscaleMinMax(-0.4, 0.4)
viz_rigid_mesh_beam.SetSmoothFaces(True)
viz_rigid_mesh_beam.SetWireframe(False)
target_mesh.AddAsset(viz_rigid_mesh_beam)

# Add mesh to the system
# cloth_mesh.SetAutomaticGravity(False)
mysystem.AddMesh(cloth_mesh)
mysystem.AddMesh(inf_mesh)
mysystem.AddMesh(target_mesh)
#
# ---------------------------------------------------------------------
# IRRLICHT
# Create an Irrlicht application to visualize the system
#
myapplication = chronoirr.ChIrrApp(mysystem, 'Cloth Simulation', chronoirr.dimension2du(1920, 1080))
myapplication.AddTypicalSky(chrono.GetChronoDataPath() + 'skybox2/')
myapplication.AddTypicalCamera(chronoirr.vector3df(bb_dz/1.2, 0., 0.))
myapplication.AddTypicalLights()
myapplication.SetPlotCollisionShapes(False)
myapplication.SetPlotCOGFrames(False)  # display coord system
myapplication.SetPlotAABB(False)
myapplication.SetShowInfos(False)

# ==IMPORTANT!== for Irrlicht to work
myapplication.AssetBindAll()
myapplication.AssetUpdateAll()
mysystem.SetupInitial()

# ---------------------------------------------------------------------
# SIMULATION
# Run the simulation

# Change the solver form the default SOR to the MKL Pardiso, more precise for fea.
msolver = mkl.ChSolverMKLcsm()
mysystem.SetSolver(msolver)
myapplication.SetTimestep(0.001)

step = 0
im_step = 0  # inverse modelling steps
threshold = 0.00007 #0.00007  # minimum value to detect stabilization
# minimum length to be reached by the optimization algorithm to detect stabilization
inv_mod_threshold = 0.001

is_inverse_modeling = False
is_detecting_stab = False
is_inv_mod_stabilized = False
target_nodes = []
target_edges = []
cloth_nodes = []
cloth_edges = []
current_cloth_nodes_pos = []

# Save first image
myapplication.SetVideoframeSave(True)

while myapplication.GetDevice().run():
    print('step', step)
    myapplication.BeginScene()
    myapplication.DrawAll()
    myapplication.DoStep()
    myapplication.EndScene()

    # Save figure of the shape only
    if step == 1:
        myapplication.SetVideoframeSave(SAVE_VIDEO)

    # Save figure once it is expanded
    if step == int(NNODES_ANGLE * NNODES_LENGTH / 4) - 1:
        myapplication.SetVideoframeSave(True)

    # Release forces after some iterations (once the mesh is completely outside of the shape)
    if step == int(NNODES_ANGLE * NNODES_LENGTH / 4):
        sleeve.release()
        shape.SetCollide(True)
        is_detecting_stab = True
        myapplication.SetVideoframeSave(False)

    # Detect stabilisation of the sleeve, ie when the sleeve has draped good enough around the shape
    # Stabilization is detected when the sleeve movements are very slow.
    # we extract the position of the resulting nodes and set this as the target shape to approximate
    if is_detecting_stab:
        print("Detecting stabilization...")

        previous_nodes = sleeve.nodes.copy()

        # Set reference position of nodes as current position, for all nodes
        sleeve.update_nodes()

        # Compute the mean standard deviation of all the nodes
        mean_sd = sleeve.get_sd_nodes(previous_nodes)

        # When it is stabilized
        if mean_sd < threshold:

            myapplication.SetVideoframeSave(True)

            # Freeze the mesh
            sleeve.freeze()

            # Remove collision detection
            shape.SetCollide(False)
            cloth_mesh.ClearContactSurfaces()
            cloth_mesh.GetAssets().pop()

            # Cut the extremities of the mesh where it is outside of the shape
            left = -bb_dz / 2.
            right = bb_dz / 2.
            target_nodes, target_edges, target_na, target_nl = sleeve.cut_extremities(left, right)

            # Downsample the mesh to match the number of sensors
            target_nodes, target_edges = downsample(target_nodes, target_edges,
                                                    target_na, target_nl,
                                                    NSENSORS_ANGLE, NSENSORS_LENGTH)

            # Visualize the target nodes (bigger nodes)
            fea_nodes = []
            noderot = chrono.ChQuaternionD(chrono.QUNIT)
            for tn in target_nodes:
                fea_node = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(tn[0], tn[1], tn[2]), noderot))
                fea_nodes.append(fea_node)
                target_mesh.AddNode(fea_node)

            # Generate the optimised shape (cloth)
            cloth_nodes, cloth_edges = gen_cylinder(bb_dx, bb_dz, NSENSORS_ANGLE, NSENSORS_LENGTH,
                                                    NEIGHBOURS, shift_z=-bb_dz / 2.)
            current_cloth_nodes_pos = cloth_nodes.copy()

            # Initial position of the nodes to optimize
            for i, cn in enumerate(cloth_nodes):
                fea_node = fea.ChNodeFEAxyzD(chrono.ChVectorD(cn[0], cn[1], cn[2]))
                fea_node.SetMass(node_mass)
                inf_mesh.AddNode(fea_node)

            is_inverse_modeling = True
            is_detecting_stab = False
            myapplication.SetTimestep(0.1)

    # Apply the optimization algorithm
    if is_inverse_modeling and len(target_nodes) > 0:

        if im_step > 0:
            #myapplication.SetVideoframeSave(False)

            print("Optimisation algorithm")
            # Apply the force optimization algorithm and visualize each iteration
            updated_nodes_pos, fvall = shapeopt_force(current_cloth_nodes_pos, cloth_edges,
                                                      target_nodes, target_edges)

            # Add a node to update visually the position of the nodes
            # Move optimised shape aside to compare shapes with the target shape
            opt_path_shape = chrono.ChPathShape()
            for i, (un, cn) in enumerate(zip(updated_nodes_pos, current_cloth_nodes_pos)):
                inf_mesh.AddNode(
                    fea.ChNodeFEAxyzD(chrono.ChVectorD(un[0], un[1], un[2])))
                # chronoirr.IVideoDriver.draw3DLine(myapplication.GetVideoDriver(),
                #                                   chronoirr.vector3df(cn[0] - (1.1 * bb_dx), cn[1], cn[2]),
                #                                   chronoirr.vector3df(un[0] - (1.1 * bb_dx), un[1], un[2]),
                #                                   chronoirr.SColor(255, 255, 0, 0))

            myapplication.AssetBindAll()
            myapplication.AssetUpdateAll()
            myapplication.DrawAll()

            current_cloth_nodes_pos = updated_nodes_pos.copy()

            # Detect stabilization of inverse modelling
            # The stabilization is reached when the edges length of the optimised shape are reaching
            # the one of the target shape.
            opt_shape_edgelen_all = geo.edgelen_all(current_cloth_nodes_pos, cloth_edges).flatten()
            target_shape_edgelen_all = geo.edgelen_all(target_nodes, target_edges).flatten()
            opt_shape_edgelen_all = [e if e else 0 for e in opt_shape_edgelen_all]
            target_shape_edgelen_all = [e if e else 0 for e in target_shape_edgelen_all]
            delta_length = np.subtract(opt_shape_edgelen_all, target_shape_edgelen_all)
            is_inv_mod_stabilized = True
            for dl in delta_length:
                print('dl', dl)
                if dl is not None:
                    if math.fabs(dl) > inv_mod_threshold:
                        is_inv_mod_stabilized = False
                        break

            if is_inv_mod_stabilized:
                is_inverse_modeling = False
                myapplication.SetVideoframeSave(False)  # Stop filming

                # Print edge length
                # opt_shape_edgelen_all = geo.edgelen_all(current_cloth_nodes_pos, cloth_edges)
                # target_shape_edgelen_all = geo.edgelen_all(target_nodes, target_edges)
                # print('opt_shape_edgelen_all')
                # print(opt_shape_edgelen_all)
                #
                # print('target_shape_edgelen_all')
                # print(target_shape_edgelen_all)

                # Move optimised shape aside to compare it with the target shape
                #for un in current_cloth_nodes_pos:
                #    rigid_mesh.AddNode(
                #        fea.ChNodeFEAxyzD(chrono.ChVectorD(un[0], un[1], un[2])))

        im_step += 1

    step += 1

## EXtract video from images:  ffmpeg -f image2 -framerate 18 -i screenshot%05d.bmp a.mp4
