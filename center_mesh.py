import os
import sys
import argparse
import glob
import open3d as o3d

def center(spine_path):
    # read mesh
    spine = o3d.io.read_triangle_mesh(spine_path)

    # get center
    centerSpine = spine.get_center()

    # move to center
    vertsSpine = spine.vertices - centerSpine
    spine.vertices = o3d.utility.Vector3dVector(vertsSpine)

    # save mesh
    o3d.io.write_triangle_mesh(spine_path.replace("_lumbar_deformed.obj", "_lumbar_deformed_centered.obj"), spine)



if __name__ == '__main__':
    arg_parser = argparse.ArgumentParser(description="Center mesh")

    arg_parser.add_argument(
        "--root_path_spines",
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

    args = arg_parser.parse_args()

    # iterate over spine IDS
    with open(args.txt_file) as file:
        spine_ids = [line.strip() for line in file]


    for spine_id in spine_ids:

        # get the paths for the segmentations of all vertebrae belonging to this spine
        unique_identifier = "*/**" + str(spine_id) + "*lumbar_deformed.obj"
        spine_mesh_paths = sorted(glob.glob(os.path.join(args.root_path_vertebrae, unique_identifier), recursive=True))

        for spine_mesh in spine_mesh_paths:
            print("Centering " + str(spine_mesh))
            center(spine_mesh)