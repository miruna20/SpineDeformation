import glob
import os
import json

def get_spines_with_lumbar_vertebrae(root_folder, file):
    # gather all of the json files to be able to check for lumbar vertebrae

    #TODO get here the version that looks recursively for files
    filenames_json = glob(os.path.join(root_folder, '*.json'))

    lumbar_vertebreae_file = open(file, "w")


    # iterate over them and check if they have L1 until L5 present
    lumbar_labels = [20,21,22,23,24]
    for json_file in filenames_json:
        contained_lumbar_vert = []
        f = open(json_file)
        data = json.load(f)
        for i in range(len(data)):
            if 20<= data[i]["label"] <= 24:
                contained_lumbar_vert.append(data[i]["label"])

        # if all lumbar vertebrae are contained
        if contained_lumbar_vert == lumbar_labels:
            lumbar_vertebreae_file.write(os.path.basename(json_file))




    # if they do then write the name of the folder (e.g sub-verse90) into the list


if __name__ == "__main__":
    root_folder = "/home/miruna20/Documents/Thesis/Dataset/VerSe2020/full_spines/subjectbased_structure/01_training"
    file = "../samples/lumbar_spines.txt"