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
        help="ImFusion workspace files that has all of the necessary algo info to transform from object to labelmap"
    )

    arg_parser.add_argument(
        "--root_path_vertebrae",
        required=True,
        dest="root_path_vertebrae",
        help="Root path to the vertebrae folders."
    )
    args = arg_parser.parse_args()
    print("Converting deformed spines from obj format to labelmaps")
    # iterate over the txt file
    with open(args.txt_file) as file:
        spine_ids = [line.strip() for line in file]

    placeholders = ['PathToFile', 'PathToSave']
    for spine_id in spine_ids:
        print("Processing: " + str(spine_id))
        look_for = "**/*" + str(spine_id) + "*deformed" + '*.obj'
        filenames = glob.glob(os.path.join(args.root_path_vertebrae, look_for), recursive=True)
        if (len(filenames) == 0):
            print("No deformed files could be found for " + str(spine_id), file=sys.stderr)
            continue

        for deformed_file_obj in filenames:
            # call imfusion console with the correct parameters
            arguments = ""
            arguments += placeholders[0] + "=" + deformed_file_obj + " "
            arguments += placeholders[1] + "=" + deformed_file_obj.replace(".obj", ".nii.gz")
            os.system("ImFusionConsole" + " " + args.workspace_file + " " + arguments)