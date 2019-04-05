import pychrono as chrono
import numpy as np


def make_ChVectorD(v):
    if len(v) != 3:
        raise ValueError('Vector v must have a length of 3 for x,y,z coordinates')
    return chrono.ChVectorD(v[0], v[1], v[2])


# Preprocess obj file to change millimeters to meters
def obj_from_millimeter(filepath, unit_factor, filename_suffix):
    """
    Change .obj files units from millimeter to the desired unit. Create a new file
    with the given extension_name
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
                ## Change millimeters to meters
                xyz *= unit_factor
                res = values[0] + " " + " ".join([f"{val:0.8f}" for val in xyz]) + '\n'
            f_mm.write(res)
    f_mm.close()
    return '/'.join(folders[:-1]) + '/' + new_file_name
