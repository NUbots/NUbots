#!/usr/bin/env python3

import os

import b
from utility.shell import WrapPty
from utility.yarn import find_package_json


def register(command):
    # Install help
    command.help = "Run yarn in a docker container"

    command.add_argument(
        "--address", dest="address", default="", help="The address that NUClearNet.js will listen to for devices"
    )
    command.add_argument("--play", dest="play", default="", help="The NBS file to playback")
    command.add_argument("args", nargs="...", help="the arguments to pass through to yarn")


def run(address, play, args, **kwargs):
    pty = WrapPty()

    # Find path to package.json
    # We need to run the yarn command from this directory
    package_path = find_package_json(project_root=b.project_dir, package_name="NUsight")

    # Format the extra arguments
    extra_args = []
    if address != "":
        extra_args.extend(["--address", address])
    if play != "":
        extra_args.extend(["--play", play])

    # Change directory and run the requested yarn command
    os.chdir(package_path)
    exit(pty.spawn(["yarn", *args, *extra_args]))
