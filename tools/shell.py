#!/usr/bin/env python3

from dockerise import WrapPty, run_on_docker

import b


@run_on_docker
def register(command):
    # Install help
    command.help = "Open an interactive shell in a docker container"

    command.add_argument(
        "args", nargs="...", help="the command to run on the docker image. If empty will default to /bin/bash"
    )


@run_on_docker
def run(args, **kwargs):
    pty = WrapPty()
    if len(args) == 0:
        exit(pty.spawn(["/bin/bash"]))
    else:
        exit(pty.spawn(args))
