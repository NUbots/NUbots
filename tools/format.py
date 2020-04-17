#!/usr/bin/env python3

import os
from subprocess import call, check_output

import b
from dockerise import run_on_docker


@run_on_docker
def register(command):
    command.help = "Format all the code in the codebase using clang-format"


@run_on_docker
def run(**kwargs):

    # Change into the project directory
    os.chdir(b.project_dir)

    # Prepend the users local bin directory to the path
    env = os.environ
    env["PATH"] = os.pathsep.join([os.path.join(env["HOME"], ".local", "bin"), env["PATH"]])

    # Use git to get all of the files that are committed to the repository
    files = check_output(["git", "ls-files"]).decode("utf-8")
    for f in files.splitlines():
        if f.endswith((".h", ".c", ".cc", ".cxx", ".cpp", ".hpp", ".ipp", ".proto")):
            print("Formatting {} with clang-format".format(f))
            call(["clang-format", "-i", "-style=file", f], env=env)
        elif f.endswith(("CMakeLists.txt", ".cmake", ".role")):
            print("Formatting {} with cmake-format".format(f))
            call(["cmake-format", "-i", f], env=env)
        elif f.endswith((".py")):
            print("Formatting {} with isort and black".format(f))
            call(["isort", f], env=env)
            call(["black", f], env=env)
