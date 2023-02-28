from glob import glob
import os


import b

def all_role_names():
    # Find all role files
    fnames = glob(os.path.join(b.project_dir, "roles", "**", "*.role"), recursive=True)
    # Strip everything from file paths except role name and subdirectory in roles/
    return [os.path.splitext(f.replace(os.path.join(b.project_dir, "roles", ""), ""))[0] for f in fnames]
