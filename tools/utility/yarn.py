#!/usr/bin/env python3

import json
import os

from termcolor import colored


def find_eslint(node_modules_path):
    # Check in the node_modules folder for eslint
    eslint_path = os.path.join(node_modules_path, ".bin", "eslint")
    if os.path.exists(eslint_path):
        return eslint_path

    print(
        colored(
            "Cannot find eslint, run `./b yarn` first if you need to format typescript files", "red", attrs=["bold"]
        )
    )
    return None


def find_package_json(project_root=".", package_name="NUsight"):
    # Find path to package.json
    # We need to run the eslint command from this directory

    # First check where we expect NUsight's package.json file to be
    nusight_root = os.path.join(project_root, "nusight2")
    if os.path.exists(os.path.join(nusight_root, "package.json")):
        return nusight_root

    # Walk the files in the project to find NUsight's package.json
    package_path = project_root
    for root, _, files in os.walk(project_root):
        for entry in files:
            if entry == "package.json":
                # Make sure this package.json belongs to the project we are looking for
                with open(os.path.join(root, entry), "r") as f:
                    try:
                        package = json.load(f)
                        if package["name"] == package_name:
                            return os.path.dirname(os.path.join(root, entry))
                    except:
                        pass

    return package_path
