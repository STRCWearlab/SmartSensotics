import pychrono as chrono
import pychrono.fea as fea
import numpy as np


def make_ChVectorD(v):
    if len(v) != 3:
        raise ValueError('Vector v must have a length of 3 for x,y,z coordinates')
    return chrono.ChVectorD(v[0], v[1], v[2])


# Preprocess obj file to change millimeters to meters
def obj_from_millimeter(filepath, unit_factor, filename_suffix):
    """
    Change .obj files units from millimeter to the desired unit. Create a new file
    with the given filename_suffix.
    :param filepath: full path to the file
    :param unit_factor: coefficient to apply on millimeters values in obj file. e.g. 0.001 to change to meters
    :param filename_suffix: append a suffix to the created filename
    :return: the full path of the converted new file
    """
    if filename_suffix == '':
        raise ValueError("You must specify a filename_suffix")
    folders = filepath.split('/')
    file = folders[-1].split('.')
    file_name = '.'.join(file[:-1])
    file_ext = file[-1]
    new_file_name = f"{file_name}{filename_suffix}.{file_ext}"
    f_mm = open('/'.join(folders[:-1]) + '/' + new_file_name, 'w+')
    with open(filepath) as f:
        for line in f:
            res = line
            if line.startswith('v'):
                values = line.split(' ')
                xyz = np.array(values[1:], dtype=np.float)
                # Change millimeters to the unit factor
                xyz *= unit_factor
                res = values[0] + " " + " ".join([f"{val:0.8f}" for val in xyz]) + '\n'
            f_mm.write(res)
    f_mm.close()
    return '/'.join(folders[:-1]) + '/' + new_file_name


def get_cylinder_radius_thickness(shape_type, params):
    if shape_type == 'DE':
        return int(params[-2]), int(params[-1])
    return int(params[-2]), int(params[-1])


def get_shape_params(shape_path):
    all = shape_path.split('/')[-1].split('.')[0].split('_')
    shape_type = all[0]
    params = all[1:]
    return shape_type, params


def get_shape_min_radius(shape_path, depth, height):
    """
    Get the minimum radius of the shape
    :param SHAPE_PATH: the filepath to the shape
    :return: the minimum radius of the shape in mm
    """
    shape_type, params = get_shape_params(shape_path)
    min_radius = min(depth, height) / 2.
    if shape_type == 'C':
        min_radius = eval(params[0])
    if shape_type == 'Cone':
        min_radius = eval(params[0])
    if shape_type == 'E':
        min_radius = 1.2*eval(params[1])
    return min_radius


def load_shape(filepath, contact_method, texture):
    """
    Import the mesh stored in filepath as a shape
    :param filepath: Path to the mesh
    :param contact_method: SMC or NSC
    :param texture: Path to the texture .jpg
    :return: A ChBody shape
    """
    shape = chrono.ChBody(contact_method)
    shape.SetBodyFixed(True)
    shape_mesh = chrono.ChObjShapeFile()
    shape_mesh.SetFilename(filepath)
    shape.AddAsset(shape_mesh)
    tmc = chrono.ChTriangleMeshConnected()
    tmc.LoadWavefrontMesh(filepath)

    # Add a collision mesh to the shape
    shape.GetCollisionModel().ClearModel()
    shape.GetCollisionModel().AddTriangleMesh(tmc, True, True)
    shape.GetCollisionModel().BuildModel()
    shape.SetShowCollisionMesh(True)
    shape.SetCollide(False)

    # Add a skin texture
    skin_texture = chrono.ChTexture()
    skin_texture.SetTextureFilename(chrono.GetChronoDataPath() + texture)
    shape.GetAssets().push_back(skin_texture)

    return shape


def build_external_cylinder(cyl_radius, shape_length, density, contact_method, offset):
    """
    Build a left and right cylinder next to the shape_lenght that must be centered
    on 0,0,0 coord system.
    :param cyl_radius: radius of the two external cylinders
    :param shape_length: lenght of the shape
    :param density: Density of the cylinders
    :param contact_method: SMC or NSC
    :param offset: distance between disk and shape
    :return: two ChEasyCylinder
    """
    height = 0.01 * shape_length
    qCylinder = chrono.Q_from_AngX(90 * chrono.CH_C_DEG_TO_RAD)
    left_cyl = chrono.ChBodyEasyCylinder(cyl_radius, height, density,
                                         True, False, contact_method)
    right_cyl = chrono.ChBodyEasyCylinder(cyl_radius, height, density,
                                          True, False, contact_method)
    left_cyl.SetRot(qCylinder)
    right_cyl.SetRot(qCylinder)
    left_cyl.SetPos(chrono.ChVectorD(0, 0, -(shape_length / 2. + offset)))
    right_cyl.SetPos(chrono.ChVectorD(0, 0, shape_length / 2. + offset))
    left_cyl.SetBodyFixed(True)
    right_cyl.SetBodyFixed(True)
    return left_cyl, right_cyl


def shift_mesh(mesh, shift_x=0, shift_y=0, shift_z=0):
    for n in mesh.GetNodes():
        p = n.GetPos()
        n.SetPos(chrono.ChVectorD(eval(p[0])+shift_x,
                                  eval(p[1])+shift_y,
                                  eval(p[2])+shift_z))


def viz_target_edges(mesh, fea_nodes, edges):
    """
    Build visual edges between the given nodes according to edges structure
    :param mesh: the mesh to atatch the elements to
    :param fea_nodes: chrono nodes
    :param edges: edges structure
    :return: chrono edges
    """
    section = fea.ChBeamSectionAdvanced()
    beam_wy = 0.01
    beam_wz = 0.01
    section.SetAsRectangularSection(beam_wy, beam_wz)
    for nidx, edge in enumerate(edges):
        print(nidx, edge)
        for n in edge:
            # nidx is the current node, n is its neighbor
            # Add an edge between nidx and n if n exists
            print('n=',n)
            if n > -1:
                e = fea.ChElementBeamEuler()
                e.SetNodes(fea_nodes[nidx], fea_nodes[n])
                e.SetSection(section)
                mesh.AddElement(e)
        if nidx==6:
            break