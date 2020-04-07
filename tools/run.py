#!/usr/bin/env python3

import os

import b
from dockerise import WrapPty, run_on_docker


@run_on_docker
def register(command):
    # Install help
    command.help = "Run a built binary within the local docker container"

    command.add_argument(
        "--asan", dest="use_asan", action="store_true", default=False, help="Add ASAN environment variables"
    )
    command.add_argument("args", nargs="+", help="the command and any arguments that should be used for the execution")


@run_on_docker
def run(args, use_asan, **kwargs):

    # Change into the build directory
    os.chdir(os.path.join(b.project_dir, "..", "build"))

    # Add 'bin/` to the command (first argument)
    args[0] = os.path.join("bin", args[0])

    # Get current environment
    env = os.environ

    # Add necessary ASAN environment variables
    if use_asan:
        env.update({"ASAN_OPTIONS": "log_path=/home/nubots/NUbots/asan.log"})


    # Run the command
    pty = WrapPty()
    exit(pty.spawn(cmd + args, env))
