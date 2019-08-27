#!/usr/bin/env python3

from dockerise import run_on_docker
import os
import pty


@run_on_docker
def register(command):
    # Install help
    command.help = "Open an interactive shell in a docker container"

    command.add_argument(
        "args", nargs="...", help="the command to run on the docker image. If empty will default to /bin/bash"
    )


@run_on_docker
def run(args, **kwargs):

    # If no arguments run bash and return its exit code
    if len(args) == 0:
        exit(pty.spawn("/bin/bash") >> 8)
    else:
        exit(pty.spawn(args) >> 8)
