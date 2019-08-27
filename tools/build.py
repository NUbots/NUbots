#!/usr/bin/env python3

from dockerise import run_on_docker
import b
import os


@run_on_docker
def register(command):
    # Install help
    command.help = "build the NUbots codebase"


@run_on_docker
def run(**kwargs):

    # Change into the build directory
    os.chdir(os.path.join(b.project_dir, "..", "build"))

    # Run cmake if we haven't already
    if not os.path.isfile("CMakeCache.txt"):
        exitcode = os.system("cmake {} -GNinja".format(b.project_dir)) >> 8

        # If cmake errors return with its status
        if exitcode != 0:
            exit(exitcode)

    # Return the exit code of ninja
    exit(os.system("ninja") >> 8)
