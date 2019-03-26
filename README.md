# SmartSensOtics

Shape sensing textile for orthotics.

## Structure
- `data` mesh files, textures
- `chrono` python files to run 3D simulations

## ProjectChrono

[ProjectChrono](https://projectchrono.org/) is an Open Source Multi-physics Simulation Engine. It is cross-platform. We use the PyChrono interface to run 3D simulation in python 3.6.

Run the simulation with `$ python chrono/cloth_simulation.py`

### Installation

 1. Install Miniconda (400 MB) or Anaconda (3 GB): https://conda.io/projects/conda/en/latest/user-guide/install/index.html
 2. Install Python 3.6 within Miniconda or Anaconda by typing `$ conda install python=3.6`
 3. Install PyChrono development branch `$ conda install -c projectchrono/label/develop pychrono`
