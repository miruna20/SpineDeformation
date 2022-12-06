import meshio
import numpy as np
import os

def vtuToObj(file_path):
    """
    Transform vtu file resulted from sofa framework to obj file
    :param file_path:
    :return:
    """
    mesh_vtu = meshio.read(file_path)

    mesh = meshio.Mesh(
        mesh_vtu.points * 1e3,
        mesh_vtu.cells,
        # Optionally provide extra data on points, cells, etc.
        mesh_vtu.point_data,
        # Each item in cell data must match the cells array
        mesh_vtu.cell_data,
    )

    dst_filename = os.path.join(file_path.replace(".vtu", ".obj"))
    mesh.write(dst_filename)

if __name__ == "__main__":
    vtuToObj("/home/miruna20/Documents/Thesis/SpineDeformation/code/sofa/build/results/spinesub-verse500_vert3_20_0.vtu")
