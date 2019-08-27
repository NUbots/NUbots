#!/usr/bin/env python3

from dockerise import run_on_docker
import b
import os
import pty


@run_on_docker
def register(command):
    # Install help
    command.help = "Open an interactive shell in a docker container"

    command.add_argument(
        "-i",
        "--interactive",
        dest="interactive",
        action="store_true",
        default=False,
        help="perform an interactive configuration using ccmake",
    )


@run_on_docker
def run(interactive, **kwargs):

    # If configure then run ccmake
    if interactive:
        print(os.path.join(b.project_dir, "..", "build"))
        os.chdir(os.path.join(b.project_dir, "..", "build"))
        pty.spawn("ccmake {} -GNinja".format(b.project_dir))
