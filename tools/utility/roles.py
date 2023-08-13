import os
from glob import glob

import b
from utility.dockerise import run_on_docker


def dir_role_names(dir):
    # Find all role files
    fnames = glob(os.path.join(b.project_dir, "roles", dir, "**", "*.role"), recursive=True)
    # Strip everything from file paths except role name and subdirectory in roles/
    return [os.path.splitext(f.replace(os.path.join(b.project_dir, "roles", ""), "").replace(os.path.sep, "-"))[0] for f in fnames]

def all_role_names():
    return dir_role_names("")

def role_folders():
    # Find all role folders
    dnames = glob(os.path.join(b.project_dir, "roles", "**/"), recursive=True)
    dnames.pop(0)
    # Strip everything from file paths except role name and subdirectory in roles/
    return [os.path.splitext(d.replace(os.path.join(b.project_dir, "roles", ""), "").replace(os.path.sep, ""))[0] for d in dnames]
