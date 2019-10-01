#!/usr/bin/env python3

import os
import b
from dockerise import run_on_docker
from subprocess import call, check_output


@run_on_docker
def register(command):
    command.help = "Format all the code in the codebase using clang-format"


@run_on_docker
def run(**kwargs):

    # Change into the project directory
    os.chdir(b.project_dir)

    # Use git to get all of the files that are committed to the repository
    files = check_output(["git", "ls-files"]).decode("utf-8")
    for f in files.splitlines():
        if f.endswith((".h", ".c", ".cc", ".cxx", ".cpp", ".hpp", ".ipp", ".proto")):
            print("Formatting {} with clang-format".format(f))
            call(["clang-format", "-i", "-style=file", f])
        elif f.endswith(("CMakeLists.txt", ".cmake", ".role")):
            print("Formatting {} with cmake-format".format(f))
            call(["cmake-format", "-i", f])
        elif f.endswith((".py")):
            print("Formatting {} with black".format(f))
            call(["black", f])
