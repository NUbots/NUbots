#!/usr/bin/env python3

import os

import b
from dockerise import WrapPty, run_on_docker


@run_on_docker
def register(command):
    # Install help
    command.help = "Run a built binary within the local docker container"

    command.add_argument("args", nargs="+", help="the command and any arguments that should be used for the execution")


@run_on_docker
def run(args, **kwargs):

    # Change into the build directory
    os.chdir(os.path.join(b.project_dir, "..", "build"))

    # Prepend the chosen editor to the command
    editor = os.environ.get("EDITOR", "nano")
    editor = editor if editor in ("nano", "vim") else "nano"
    args = [editor, *args]

    # Run the command
    pty = WrapPty()
    exit(pty.spawn(args))
