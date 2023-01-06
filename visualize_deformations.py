import os
import argparse
import glob
import sys

if __name__ == "__main__":
    arg_parser = argparse.ArgumentParser(description="Generate strings in between vertebrae for spine deformation")

    arg_parser.add_argument(
        "--list_file_names",
        required=True,
        dest="txt_file",
        help="Txt file that contains all spines that contain all lumbar vertebrae"
    )

    arg_parser.add_argument(
        "--workspace_file",
        required=True,
        dest="workspace_file",
        help="ImFusion workspace files that contains the visualization algorithm of multiple meshes"
    )

    arg_parser.add_argument(
        "--root_path_spines",
        required=True,
        dest="root_path_spines",
        help="Root path to the spines folders."
    )

    arg_parser.add_argument(
        "--nr_deform_per_spine",
        required=True,
        dest="nr_deform_per_spine",
        help="Number of deformations per spine"
    )

    args = arg_parser.parse_args()

    # iterate over spine IDS
    with open(args.txt_file) as file:
        spine_ids = [line.strip() for line in file]

    placeholders = ['PathToInitialLumbarSpine', 'PathToDeformedLumbarSpine']
    for spine_id in spine_ids:

        # find original vertebrae as obj files
        look_for = "**/*" + str(spine_id) + '_lumbar_msh.obj'
        filename_undeformed_spine = sorted(glob.glob(os.path.join(args.root_path_spines, look_for), recursive=True))

        if (len(filename_undeformed_spine) != 1):
            print("More or less than 1 spine was found for " + str(spine_id),
                  file=sys.stderr)
            continue

        for deform in range(int(args.nr_deform_per_spine)):
            # find deformed vertebrae as obj files
            print("Visualizing: " + str(spine_id) + " deformation: " +  str(deform))
            look_for = "**/*" + str(spine_id) + "*forcefield" + str(deform) + "*deformed" + '*.obj'
            filename_deformed_spine = sorted(glob.glob(os.path.join(args.root_path_spines, look_for), recursive=True))

            if (len(filename_deformed_spine) != 1):
                print("More or less than 1 spine was found for " + str(spine_id) + " and deformation:" + str(deform), file=sys.stderr)
                continue
            arguments = ""

            # add paths of the original spine
            arguments += placeholders[0] + "=" + filename_undeformed_spine[0] + " "

            # add paths of the deformed vertebrae
            arguments += placeholders[1] + "=" + filename_deformed_spine[0] + " "

            # call ImFusionSuite and visualize the initial lumbar spine vs the deformed spine
            os.system("ImFusionSuite" + " " + args.workspace_file + " " + arguments)
