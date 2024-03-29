import subprocess
import argparse

if __name__ == '__main__':
    """
    Pipeline for spine deformation
    #1. Select spines that have all lumbar vertebrae present and write the spines id in a txt file
    #2. Generate springs in between vertebra body and facets as well as fixed points. These serve for the spine deformation
    #3. Run deformation script (call sofa from python environment) and generate the deformed spines, save them as vtu (one vtu/vertebra) 
    #4. Convert vtu to obj
    #5. Merge all obj files from one spine into one mesh file. 
    #5. Run Imfusion workspace file and compute labelmaps from obj files (will further be used for ultrasound simulation)
    
    Folder structure for this pipeline:
    - root_path_spines directory:
        <root_path_spines>/<spine_id>/<spine_id>*.json --> json files are required to determine which spine_ids contain lumbar vertebrae
        
    - root_path_vertebrae:
        <root_path_vertebrae>/<spine_id>/<spine_id>*_msh.obj --> mesh files of individual vertebrae are used for deformation
        --> to separate spine segmentations into vertebrae segmentations and transform segmentation to mesh check 
            - "https://github.com/miruna20/thesis/blob/main/separate_spine_into_vertebrae.py"
            - "https://github.com/miruna20/thesis/blob/main/convert_segmentation_into_mesh.py"
    """

    #TODO pipe the results to a log txt file
    #TODO measure the running time of each step individually

    arg_parser = argparse.ArgumentParser(description="Generate strings in between vertebrae for spine deformation")

    arg_parser.add_argument(
        "--root_path_spines",
        required=True,
        dest="root_path_spines",
        help="Root path to the vertebrae folders."
    )

    arg_parser.add_argument(
        "--root_path_vertebrae",
        required=True,
        dest="root_path_vertebrae",
        help="Root path to the vertebrae folders."
    )

    arg_parser.add_argument(
        "--list_file_names",
        required=True,
        dest="txt_file",
        help="Txt file that contains all spines that contain all lumbar vertebrae"
    )

    arg_parser.add_argument(
        "--root_folder_json_files",
        required=False,
        dest="root_json_files",
        help="Root folder where the json files will be saved."
    )
    arg_parser.add_argument(
        "--workspace_file_merge_obj_files",
        required=True,
        dest="workspace_file_merge_obj_files",
        help="ImFusion workspace files that has all of the necessary algo info to merge 5 obj files into one"
    )

    arg_parser.add_argument(
        "--nr_deform_per_spine",
        required=True,
        dest="nr_deform_per_spine",
        help="Number of deformed spines per initial spine."
    )
    arg_parser.add_argument(
        "--root_folder_forces_files",
        required=False,
        dest="forces_folder",
        help="Root folder where the txt files with force fields will be saved."
    )

    arg_parser.add_argument(
        "--pipeline",
        nargs='+',
        default=['all'],
        help="Specify the steps of the pipeline that will be executed "
    )

    args = arg_parser.parse_args()

    root_path_spines = args.root_path_spines
    root_path_vertebrae = args.root_path_vertebrae
    txt_file_lumbar_spines = args.txt_file
    root_folder_json_files = args.root_json_files
    workspace_file_merge_obj_files = args.workspace_file_merge_obj_files
    nr_deform_per_spine = args.nr_deform_per_spine
    forces_folder = args.forces_folder

    pipeline = args.pipeline
    if 'select_lumbar_spines' in pipeline or 'all' in pipeline:
        subprocess.run(['python', 'get_spines_lumbar_vertebrae.py',
                        '--root_path_spines', root_path_spines,
                        '--list_file_names', txt_file_lumbar_spines])


    if 'scale_mesh_down' in pipeline or 'all' in pipeline:
        subprocess.run(['python', 'scale_mesh_down.py',
                        '--root_path_vertebrae', root_path_vertebrae,
                        '--list_file_names', txt_file_lumbar_spines,
                        '--workspace_scale_mesh', "scale_down_mesh.iws"])
    if 'generate_springs' in pipeline or 'all' in pipeline:
        subprocess.run(['python', 'generate_springs_spine_deformation.py',
                        '--root_path_vertebrae', root_path_vertebrae,
                        '--root_folder_json_files', root_folder_json_files,
                        '--list_file_names', txt_file_lumbar_spines
                        ])

    # deforms all spines in the list without GUI
    # works when called from an environment with python version matching the one from the installation of sofa
    # in my case python 3.9
    # QT and therefore the GUI might not work if QT is not installed in the same environment
    # for deforming all vertebrae, the GUI is not needed
    if 'deform_spines' in pipeline or 'all' in pipeline:
        subprocess.run(['python', 'lumbar_spine_deformation_scene.py',
                        '--root_path_vertebrae', root_path_vertebrae,
                        '--list_file_names', txt_file_lumbar_spines,
                        '--root_folder_json_files', root_folder_json_files,
                        '--nr_deform_per_spine', nr_deform_per_spine,
                        '--root_folder_forces_files', forces_folder,
                        '--deform_all'
                        ])
    # TODO figure out why sofa gets a malloc error after fininshing the simulation
    # TODO otherwise the pipeline is blocked here
    if 'convert_vtu_to_obj' in pipeline or 'all' in pipeline:
        subprocess.run(['python', 'convert_vtu_to_obj.py',
                        '--list_file_names', txt_file_lumbar_spines,
                        '--root_path_vertebrae', root_path_vertebrae
                        ])
    if 'scale_mesh_up' in pipeline or 'all' in pipeline:
        subprocess.run(['python', 'scale_mesh_up.py',
                        '--root_path_vertebrae', root_path_vertebrae,
                        '--list_file_names', txt_file_lumbar_spines,
                        '--workspace_scale_mesh', "scale_up_mesh.iws"])
    if 'merge_vertebrae_into_spine' in pipeline or 'all' in pipeline:
        subprocess.run(['python', "merge_vertebrae_into_spine_mesh.py",
                        '--list_file_names', txt_file_lumbar_spines,
                        '--workspace_file', workspace_file_merge_obj_files,
                        '--root_path_spines', root_path_spines,
                        '--root_path_vertebrae', root_path_vertebrae,
                        '--nr_deform_per_spine', nr_deform_per_spine
                        ])
    if 'center_spine_and_vertebrae' in pipeline or 'all' in pipeline:
        subprocess.run(['python', 'center_mesh.py',
                        '--root_path_spines', root_path_spines,
                        '--root_path_vertebrae', root_path_vertebrae,
                        '--list_file_names', txt_file_lumbar_spines,
                        '--nr_deform_per_spine', nr_deform_per_spine])