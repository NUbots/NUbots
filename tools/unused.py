#!/usr/bin/env python3

import os
import re
import b

def register(command):
    # Install help
    command.help = "Creates a list of unused or commented out modules"


def run(**kwargs):

    source_dir = b.cmake_cache[b.cmake_cache["CMAKE_PROJECT_NAME"] + "_SOURCE_DIR"]
    roles_path = os.path.join(source_path + b.cmake_cache['NUCLEAR_ROLE_DIR'])
    modules_path = os.path.join(source_path, b.cmake_cache['NUCLEAR_MODULE_DIR'])
    existing_modules = {}
    missing_modules = {}
    commented_modules = {}
    used_modules = []
    unused_commented_modules = []

    # a list of folders that could be within a module
    skip_list = ["src", "data", "config", "tests", "report"]
    skip_list = [s + os.sep for s in skip_list]

    # find all the modules that are available
    for root, dirs, files in os.walk(modules_path):
        if len(files) != 0:
            if not any(substring in (root + os.sep) for substring in skip_list):
                existing_modules[
                    root.replace(modules_path + os.sep, "")
                ] = 0  # make sure we remove the base path from the module

    # compare our roles to our existing modules
    for root, dirs, files in os.walk(roles_path):
        for file in files:
            with open(os.path.join(roles_path, file), "r") as role:
                for line in role:
                    line = line.strip()

                    if "#" in line:  # a commented
                        line = re.sub(r"##+", "#", line)  # remove multiple # that are next to each other
                        location = line.find("#")

                        if location == 0:
                            line = line[1:]  # remove the comment and continue to process
                            if "::" in line:  # the commented line is a commented out module
                                additional_comment_loc = line.find("#")
                                if additional_comment_loc > 0:
                                    line = line[:additional_comment_loc]

                                line = line.replace("::", os.sep)
                                commented_modules[line.strip()] = 1
                            continue
                        else:
                            # the comment was just a description, remove the comment and continue
                            line = line[:location].strip()

                    if "::" in line:  # the line is a module
                        line = line.replace("::", os.sep)
                        if line in existing_modules:
                            existing_modules[line] += 1
                            used_modules.append(line)
                        else:
                            missing_modules[line] = 1

    print("Unused modules:\n")
    for key in sorted(existing_modules.keys()):
        if existing_modules[key] == 0:
            if key not in commented_modules.keys():
                print("\t", key)
            else:
                unused_commented_modules.append(key)

    print("\nUnused but commented out modules:\n")
    for key in sorted(commented_modules):
        if key not in used_modules:
            if key in existing_modules:
                if existing_modules[key] > 0:
                    continue

            print("\t", key)
