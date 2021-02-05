#!/usr/bin/env python3

import multiprocessing
import os
import sys
from functools import partial
from subprocess import STDOUT, CalledProcessError, check_output

import b
from dockerise import run_on_docker
from utility.yarn import find_eslint, find_package_json


def _do_format(path, eslint_path, package_path):
    text = ""
    try:
        # Use the absolute path to file
        abs_path = os.path.abspath(path)

        if path.endswith((".h", ".c", ".cc", ".cxx", ".cpp", ".hpp", ".ipp", ".proto", ".frag", ".glsl", ".vert")):
            text = "Formatting {} with clang-format\n".format(path)
            text = text + check_output(["clang-format", "-i", "-style=file", abs_path], stderr=STDOUT).decode("utf-8")
        elif path.endswith(("CMakeLists.txt", ".cmake", ".role")):
            text = "Formatting {} with cmake-format\n".format(path)
            text = text + check_output(["cmake-format", "-i", abs_path], stderr=STDOUT).decode("utf-8")
        elif path.endswith((".py")):
            text = "Formatting {} with isort and black\n".format(path)
            text = text + check_output(["isort", abs_path], stderr=STDOUT).decode("utf-8")
            text = text + check_output(["black", abs_path], stderr=STDOUT).decode("utf-8")
        elif eslint_path is not None and (path.endswith((".ts")) or path.endswith((".tsx"))):
            text = "Formatting {} with eslint and prettier\n".format(path)
            text = text + check_output(
                [eslint_path, "--color", "--fix", abs_path], cwd=package_path, stderr=STDOUT
            ).decode("utf-8")
    except CalledProcessError as e:
        text = text + e.output.decode("utf-8")

    return text


@run_on_docker
def register(command):
    command.help = "Format all the code in the codebase using clang-format"


@run_on_docker
def run(**kwargs):
    # Check for eslint in node_modules folder
    eslint_path = find_eslint(os.path.join(b.project_dir, "nusight2", "node_modules"))

    # Find path to package.json
    # We need to run the eslint command from this directory
    package_path = find_package_json(project_root=b.project_dir, package_name="NUsight")

    # Change into the project directory
    os.chdir(b.project_dir)

    # Use git to get all of the files that are committed to the repository
    files = check_output(["git", "ls-files"]).decode("utf-8")

    with multiprocessing.Pool(multiprocessing.cpu_count()) as pool:
        for r in pool.imap_unordered(
            partial(_do_format, eslint_path=eslint_path, package_path=package_path), files.splitlines()
        ):
            sys.stdout.write(r)
