import yaml
import json
import os 
import numpy as np

data_folder = "./all_data_300"

for file_any in os.listdir(data_folder):
    if not "yaml" in file_any:
        continue
    file_lens = os.path.join(data_folder, file_any)
    f = open(file_lens)
    data = json.load(f)

    width = 1280
    projection = "EQUISOLID"
    centre = data["lens"]["centre"]
    centre = [x * width for x in centre]
    
    k = [0.46673390301927425 / (width**2), 0.15017118169604132 / (width**4)]
    fov = data["lens"]["fov"]
    Hco = data["Hcw"]
    focal_length = data["lens"]["focal_length"] * width

    # rOCc = np.array([Hco[3][0], Hco[3][1], Hco[3][2]])
    rCOc = np.array([-Hco[3][0], -Hco[3][1], -Hco[3][2]])

    Roc = np.array([
        [Hco[0][0], Hco[0][1], Hco[0][2]],
        [Hco[1][0], Hco[1][1], Hco[1][2]],
        [Hco[2][0], Hco[2][1], Hco[2][2]]
    ])
    
    rCOo = np.matmul(Roc, rCOc).tolist()

    # Hoc = [
    #     [Hco[0][0], Hco[0][1], Hco[0][2], rCOo[0]],
    #     [Hco[1][0], Hco[1][1], Hco[1][2], rCOo[1]],
    #     [Hco[2][0], Hco[2][1], Hco[2][2], rCOo[2]],
    #     [0, 0, 0, 1],   
    # ]
    
    Hoc = [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.8],
        [0, 0, 0, 1],   
    ]

    camera_data = {
        "projection": projection,
        "focal_length": focal_length,
        'centre': centre,
        "k": k,
        "fov": fov,
        "Hoc": Hoc,
    }

    with open(file_lens, 'w') as file:
        yaml.safe_dump(camera_data, file, default_flow_style=True)