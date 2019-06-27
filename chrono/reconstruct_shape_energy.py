import pychrono as chrono
import pychrono.fea as fea
from scipy.optimize import fmin_bfgs
from scipy.optimize import minimize
import pandas as pd
from shape_energy import shape_energy
import numpy as np
from gen_cylinder import gen_cylinder
import pychrono.irrlicht as chronoirr
import pychrono.mkl as mkl
import chrono_utils as tool
import math
import geo

# Global
XLSX_FILE = "./../data/sensors/Polynomial3cone3.xlsx"
#XLSX_FILE = "./../data/sensors/Polynomial1b.xlsx"
USE_NEGATIVE_VALUE = False
#SHAPE_PATH = 'shapes/printed_April18/Cone_32.obj'
SHAPE_PATH = 'shapes/printed_April18/CCone_325.obj'

# Global -- Textile
SENSOR_LEN = 10.  # in mm
RING_GAP = 12.5  # in mm
SENSORS_PER_RING = 8
N_RINGS = 8
SLEEVE_THICKNESS = 4.215  # in mm

# Global -- Optimisation algorithm
NEIGHBOURS = 2
INFERRED_RADIUS = 60.  # in mm
INFERRED_LENGTH = (N_RINGS - 1) * RING_GAP  # Constraint of the textile

# Global -- Dont touch
UNIT_FACTOR = 0.01
metrics = ['mm', 'cm', 'dm', 'm']
metric = metrics[int(math.fabs(round(math.log(UNIT_FACTOR, 10))))]
filename = SHAPE_PATH.split('/')[-1].split('.')[0]

# ---------------------------------------------------------------------------
# Chrono setup
chrono.SetChronoDataPath("../data/")
mysystem = chrono.ChSystemSMC()
mysystem.Set_G_acc(chrono.ChVectorD(0., 0., 0.))  # Remove gravity
contact_method = chrono.ChMaterialSurface.SMC

# Shape
filepath = tool.obj_from_millimeter(chrono.GetChronoDataPath() + SHAPE_PATH, UNIT_FACTOR, f"_{metric}")
shape = tool.load_shape(filepath, contact_method, 'textures/skin.jpg')

# Get shape bounding box dimensions
bbmin, bbmax = chrono.ChVectorD(), chrono.ChVectorD()
shape.GetTotalAABB(bbmin, bbmax)
bbmin, bbmax = eval(str(bbmin)), eval(str(bbmax))
print(bbmin, bbmax)
print(shape.GetPos())
bb_dx = bbmax[0] - bbmin[0]
bb_dy = bbmax[1] - bbmin[1]
bb_dz = bbmax[2] - bbmin[2]
# Align shape to the center of axis system
shape.SetPos(chrono.ChVectorD(-bb_dx / 2. - bbmin[0],
                              -bb_dy / 2. - bbmin[1],
                              -bb_dz / 2. - bbmin[2]))
mysystem.Add(shape)

# Inferred mesh
inf_mesh_init = fea.ChMesh()
inf_mesh = fea.ChMesh()
inf_mesh_big = fea.ChMesh()

# Chrono visualization
viz_inf_mesh_init = fea.ChVisualizationFEAmesh(inf_mesh_init)
viz_inf_mesh_init.SetWireframe(True)
viz_inf_mesh_init.SetFEMglyphType(fea.ChVisualizationFEAmesh.E_GLYPH_NODE_DOT_POS)
viz_inf_mesh_init.SetSymbolsThickness(1. * UNIT_FACTOR)
inf_mesh_init.AddAsset(viz_inf_mesh_init)
mysystem.AddMesh(inf_mesh_init)

viz_inf_mesh = fea.ChVisualizationFEAmesh(inf_mesh)
viz_inf_mesh.SetWireframe(True)
viz_inf_mesh.SetFEMglyphType(fea.ChVisualizationFEAmesh.E_GLYPH_NODE_DOT_POS)
viz_inf_mesh.SetSymbolsThickness(0.5 * UNIT_FACTOR)
inf_mesh.AddAsset(viz_inf_mesh)
mysystem.AddMesh(inf_mesh)

viz_inf_mesh_big = fea.ChVisualizationFEAmesh(inf_mesh_big)
viz_inf_mesh_big.SetWireframe(True)
viz_inf_mesh_big.SetFEMglyphType(fea.ChVisualizationFEAmesh.E_GLYPH_NODE_DOT_POS)
viz_inf_mesh_big.SetSymbolsThickness(2. * UNIT_FACTOR)
inf_mesh_big.AddAsset(viz_inf_mesh_big)
mysystem.AddMesh(inf_mesh_big)

# ---------------------------------------------------------------------------
# OPTIMISATION
# Get the target edge lengths
raw = pd.read_excel(XLSX_FILE)
elongations = raw.iloc[(0 if USE_NEGATIVE_VALUE else 1), :].values[1:]
rigid_parts = raw.iloc[2, :].values[1:]
edge_t_len = [SENSOR_LEN + e + r for (e, r) in zip(elongations, rigid_parts)]

# Create the inferred mesh
nodes_inf, edges_inf = gen_cylinder(INFERRED_RADIUS, INFERRED_LENGTH,
                                    SENSORS_PER_RING, N_RINGS, NEIGHBOURS,
                                    shift_z=(-bb_dz / 2.3) / UNIT_FACTOR
                                    )

# Remove duplicate edges in inferred shape
edges_inf = edges_inf[:, np.arange(0, NEIGHBOURS - 1, 2)]

# Apply the energy algorithm on each ring
nodes_inf_all = []
# As we removed duplicate edges, we need to update the neighbourhood
new_neighbourhood = int(NEIGHBOURS / 2)
methods = ['Nelder-Mead', 'BFGS', 'Powell', 'CG', '‘L-BFGS-B', 'TNC']
method = methods[1]

nodes_inf_final = []
nodes_inf_final_unit = []
fopts = []
edges_inf_len_mat = []
iterations = []
messages = []
successes = []
for ring in range(N_RINGS):

    # For each ring
    nodes_inf_r = []
    _from = ring * SENSORS_PER_RING
    _to = (ring + 1) * SENSORS_PER_RING
    nodes_inf_init_r = nodes_inf[_from:_to]
    edges_inf_r = edges_inf[_from:_to]
    edges_inf_r = edges_inf_r - int(ring*SENSORS_PER_RING)
    edge_t_len_r = edge_t_len[_from:_to]

    # Apply energy on a single ring
    retlist = minimize(lambda x: shape_energy(x, edges_inf_r, edge_t_len_r, new_neighbourhood),
                       nodes_inf_init_r,
                       method=method,
                       callback=lambda nodes1d: {
                           print('callback iteration'),
                           nodes_inf_r.append(nodes1d * UNIT_FACTOR)
                       },
                       options={'maxiter': None, 'return_all': True})
    nodes_inf_final.append(retlist.x)
    nodes_inf_all.append(nodes_inf_r)
    fopts.append(retlist.fun)
    iterations.append(retlist.nit)
    messages.append(retlist.message)
    successes.append(retlist.success)

    nodes_inf_final_r = np.reshape(retlist.x, (int(len(retlist.x) / 3), 3))
    nodes_inf_final_r_unit = nodes_inf_final_r * UNIT_FACTOR
    nodes_inf_final_unit.append(nodes_inf_final_r_unit)

    # Get inferred edges length
    edges_inf_len_r = geo.edgelen_all(nodes_inf_final_r, edges_inf_r, new_neighbourhood)
    edges_inf_len_mat.append(edges_inf_len_r.flatten())

# Compute inferred radius
radiuses = [sum(ring) / (2. * math.pi) for ring in edges_inf_len_mat]

print(radiuses)

# Save final values of experiment
with open(f"./../data/reconstruction/{filename}_inf_{method}.txt", 'w') as f:
    f.write("nodesxyz_mm:" + ','.join([f"{n:.6f}" for row in nodes_inf_final for n in row]) + '\n')
    f.write("edgeslen_mm:" + ','.join([f"{e:.6f}" for row in edges_inf_len_mat for e in row]) + '\n')
    f.write("radiuses_mm:" + ','.join([f"{r:.6f}" for r in radiuses]) + '\n')
    # f.write("grad_min:" + ','.join([f"{g:.6f}" for g in retlist[2]]) + '\n')
    f.write("fopt:" + ','.join([f"{fopt:.6f}" for fopt in fopts]) + '\n')
    f.write("iterations:" + ','.join([f"{ite}" for ite in iterations]) + '\n')
    f.write("message:" + ','.join([f"{m}" for m in messages]) + '\n')
    f.write("success:" + ','.join([f"{s}" for s in successes]) + '\n')

# with open(f"./../data/reconstruction/{filename}_real.txt", 'w') as f:
#     offset = bb_dz / 2.3
#     H = eval('_'.split(filename)[-1]) / math.tan(math.pi / 180.)
#     f.write("nodes_mm:" + ','.join([f"{n:.6f}" for n in nodes_inf_final]) + '\n')

# ---------------------------------------------------------------------
# IRRLICHT
# Create an Irrlicht application to visualize the system
#
myapplication = chronoirr.ChIrrApp(mysystem, 'Reconstruction shape',
                                   chronoirr.dimension2du(720, 540))
myapplication.AddTypicalSky(chrono.GetChronoDataPath() + 'skybox2/')
myapplication.AddTypicalCamera(chronoirr.vector3df(1.3*bb_dz, 0., 0.))
myapplication.AddTypicalLights()
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
myapplication.SetTimestep(0.0001)

step = 0
save_at = 2
myapplication.SetVideoframeSave(False)
inf_mesh_big.AddNode(fea.ChNodeFEAxyz(chrono.ChVectorD(100., 0., 0.)))  # hack to fix issue with doStep
while myapplication.GetDevice().run():
    print('step', step)
    myapplication.BeginScene()
    myapplication.DrawAll()

    if step == save_at:
        myapplication.SetVideoframeSave(False)
        # shape.SetPos(chrono.ChVectorD(100.,0.,0.))  # hack to hide cone

    #if step > save_at and step < len(nodes_inf_all) + save_at:
    #    for ring in range(N_RINGS):
    #        nodes3d = np.reshape(nodes_inf_all[ring][step - save_at], (int(len(nodes_inf_all[ring][step - save_at]) / 3), 3))
    #        for n3 in nodes3d:
    #            fn3 = fea.ChNodeFEAxyz(tool.make_ChVectorD(n3))
    #            inf_mesh.AddNode(fn3)

    if step == len(nodes_inf_all) + save_at + 1:
        for ni in nodes_inf:
            fni = fea.ChNodeFEAxyz(tool.make_ChVectorD(ni * UNIT_FACTOR))
            inf_mesh_init.AddNode(fni)
        for ring in range(N_RINGS):
            for nf in nodes_inf_final_unit[ring]:
                fnf = fea.ChNodeFEAxyz(tool.make_ChVectorD(nf))
                inf_mesh_big.AddNode(fnf)
        myapplication.SetVideoframeSave(True)
    if step == len(nodes_inf_all) + save_at + 4:
        myapplication.SetVideoframeSave(False)

    myapplication.DoStep()
    myapplication.EndScene()

    step += 1
