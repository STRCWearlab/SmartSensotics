# SmartSensOtics

Shape sensing textile for orthotics.

## Structure
- `data` mesh files, textures
- `chrono` python files to run 3D simulations

## Shapes naming convention
The shapes files are named `{SHAPE_TYPE}_PARAM[_PARAM].obj` where `SHAPE_TYPE` is the type of the shape and `PARAM` a parameter defining that shape (like the radius). The following table describes the naming convention:

| SHAPE_TYPE | Description | PARAM | Center of Cyl. | Example |
|---|---|---|---|---|
| DE | Double Ellipsis  | r1_r2_r3_cylradius_cylthickness in [mm] | [r1,cylradius] | DE_12_7_8.obj |
| CB | Cylinder Bump | r_cylradius_thickness in [mm] | [r,r] | CB_12_10_2.obj |

## ProjectChrono

[ProjectChrono](https://projectchrono.org/) is an Open Source Multi-physics Simulation Engine. It is cross-platform. We use the PyChrono interface to run 3D simulation in python 3.6.

Run the simulation with `$ python chrono/cloth_simulation.py`

### Installation

 1. Install Miniconda (400 MB) or Anaconda (3 GB): https://conda.io/projects/conda/en/latest/user-guide/install/index.html
 2. Install Python 3.6 within Miniconda or Anaconda by typing `$ conda install python=3.6`
 3. Install PyChrono development branch `$ conda install -c projectchrono/label/develop pychrono`
