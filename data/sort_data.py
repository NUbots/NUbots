import os
import shutil

parent_paths = ["./1", "./2", "./3", "./4"]
new_path = "./all_data_300"

num_data = 300

config = []
images = []


for parent_path in parent_paths:
    for root, dirs, files in os.walk(parent_path):
        for f in files:
            if ".json" in f:
                config.append(os.path.join(parent_path, f))
            elif ".jpg" in f:
                images.append(os.path.join(parent_path, f))

if len(config) != len(images):
    print("Error, different amount of config files and images.")
    exit()

if not(os.path.isdir(new_path)):
        os.mkdir(new_path)

total_data = len(config)
skip = int(total_data / num_data)
print(total_data)

count = 0
for i in range(len(config)):
    if i % skip == 0:
        c_path = "lens" + str(count).rjust(7, "0") + ".yaml"
        shutil.copy(config[i], os.path.join(new_path, c_path))
        i_path = "image" + str(count).rjust(7, "0") + ".jpg"
        shutil.copy(images[i], os.path.join(new_path, i_path))
        count += 1
