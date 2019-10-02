#!/usr/bin/env python3

from dockerise import run_on_docker
import b
import os
import pty


@run_on_docker
def register(command):
    # Install help
    command.help = "Run a built binary within the local docker container"

    command.add_argument("args", nargs="+", help="the command and any arguments that should be used for the execution")


@run_on_docker
def run(args, **kwargs):

    # Change into the build directory
    os.chdir(os.path.join(b.project_dir, "..", "build"))

    # Add 'bin/` to the command (first argument)
    args[0] = os.path.join("bin", args[0])

    # Run the command
    exit(pty.spawn(args) >> 8)
