import os
import sys
import argparse
import glob
import open3d as o3d

def center(spine_path, vert_paths):
    # read mesh
    spine = o3d.io.read_triangle_mesh(spine_path)

    # get center
    centerSpine = spine.get_center()

    # move to center
    vertsSpine = spine.vertices - centerSpine
    spine.vertices = o3d.utility.Vector3dVector(vertsSpine)

    # save mesh
    o3d.io.write_triangle_mesh(spine_path.replace("_lumbar_deformed.obj", "_lumbar_deformed_centered.obj"), spine)

    # move also all of the vertebrae meshes to the center
    for vert_path in vert_paths:
        curr_vert = o3d.io.read_triangle_mesh(vert_path)
        vertices_curr_vert = curr_vert.vertices - centerSpine
        curr_vert.vertices = o3d.utility.Vector3dVector(vertices_curr_vert)
        o3d.io.write_triangle_mesh(vert_path.replace("_deformed_", "_deformed_centered_"),curr_vert)


if __name__ == '__main__':
    arg_parser = argparse.ArgumentParser(description="Center mesh")

    arg_parser.add_argument(
        "--root_path_spines",
        required=True,
        dest="root_path_spines",
        help="Root path of the spine folders"
    )

    arg_parser.add_argument(
        "--root_path_vertebrae",
        required=True,
        dest="root_path_vertebrae",
        help="Root path of the spine folders"
    )

    arg_parser.add_argument(
        "--list_file_names",
        required=True,
        dest="txt_file",
        help="Txt file that contains all spines that contain all lumbar vertebrae"
    )

    arg_parser.add_argument(
        "--nr_deform_per_spine",
        required=True,
        dest="nr_deform_per_spine",
        help="Number of deformed spines per initial spine."
    )

    args = arg_parser.parse_args()

    # iterate over spine IDS
    with open(args.txt_file) as file:
        spine_ids = [line.strip() for line in file]


    for spine_id in spine_ids:
        for deform in range(int(args.nr_deform_per_spine)):
            print("Centering the spine and vertebrae of: " + str(spine_id) + "and deform " + str(deform))
            # get the paths to the spine with current spine_id and deformation number
            unique_identifier_spine = "*/**" + str(spine_id)  + "*forcefield" +str(deform) +  "*lumbar_deformed.obj"
            spine_mesh_path = sorted(glob.glob(os.path.join(args.root_path_spines, unique_identifier_spine), recursive=True))

            if (len(spine_mesh_path) != 1):
                print("None or more than one mesh was found for spine: " + str(spine_id) + "and deform: " + str(deform), file=sys.stderr)
                continue
            spine_mesh_path = spine_mesh_path[0]


            unique_identifier_vert = "*/**" + str(spine_id) + "*forces" + str(deform) + "_deformed_" + "*.obj"
            # find paths of the deformed vertebrae
            vertebrae_mesh_paths = sorted(glob.glob(os.path.join(args.root_path_vertebrae, unique_identifier_vert), recursive=True))

            if (len(vertebrae_mesh_paths) != int(args.nr_deform_per_spine)):
                print("The number of vertebrae meshes should match the number of deformations per spine for : " + str(spine_id) + "and deform: " + str(deform),
                      file=sys.stderr)
                continue

            center(spine_mesh_path, vertebrae_mesh_paths)