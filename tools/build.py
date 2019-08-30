#!/usr/bin/env python3

from dockerise import run_on_docker
import b
import os
import subprocess


@run_on_docker
def register(command):
    # Install help
    command.help = "build the NUbots codebase"

    command.add_argument("args", nargs="...", help="the arguments to pass through to ninja")


@run_on_docker
def run(args, **kwargs):

    # Change into the build directory
    os.chdir(os.path.join(b.project_dir, "..", "build"))

    # Run cmake if we haven't already
    if not os.path.isfile("build.ninja"):
        exitcode = os.system("cmake {} -GNinja".format(b.project_dir)) >> 8

        # If cmake errors return with its status
        if exitcode != 0:
            exit(exitcode)

    # Return the exit code of ninja
    exit(subprocess.call(["ninja", *args]))
