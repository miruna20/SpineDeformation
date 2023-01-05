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
        help="ImFusion workspace files that has all of the necessary algo to merge multiple object files into one"
    )

    arg_parser.add_argument(
        "--root_path_vertebrae",
        required=True,
        dest="root_path_vertebrae",
        help="Root path to the vertebrae folders."
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

    placeholders = ['PathToL20', 'PathToL21', 'PathToL22', 'PathToL23', 'PathToL24', 'PathToSave']
    for spine_id in spine_ids:
        for deform in range(int(args.nr_deform_per_spine)):
            print("Processing: " + str(spine_id) + " deformation: " +  str(deform))
            look_for = "**/*" + str(spine_id) + "*forces" + str(deform) + "deformed" + '*.obj'
            filenames = sorted(glob.glob(os.path.join(args.root_path_vertebrae, look_for), recursive=True))
            if (len(filenames) != 5):
                print("More or less than 5 vertebrae were found for " + str(spine_id) + " and deformation:" + str(deform), file=sys.stderr)
                continue

            arguments = ""
            for vert_lev in range(5):
                arguments += placeholders[vert_lev] + "=" + filenames[vert_lev] + " "

            # add the output path as parameter
            arguments += placeholders[5] + "=" + os.path.join(args.root_path_spines,spine_id,spine_id + "forcefield" + str(deform) +"_lumbar_deformed.obj")
            print(arguments)
            # call imfusion console
            os.system("ImFusionConsole" + " " + args.workspace_file + " " + arguments)