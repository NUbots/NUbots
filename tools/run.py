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
    command.add_argument(
        "--gdb", dest="use_gdb", action="store_true", default=False, help="Run the specified program using gdb"
    )
    command.add_argument(
        "--valgrind",
        dest="use_valgrind",
        action="store_true",
        default=False,
        help="Run the specified program using valgrind",
    )
    command.add_argument("args", nargs="+", help="the command and any arguments that should be used for the execution")


@run_on_docker
def run(args, use_asan, use_gdb, use_valgrind, **kwargs):

    if use_gdb and use_valgrind:
        raise Exception("Cannot run with both gdb and valgrind")
    if use_asan and use_valgrind:
        raise Exception("Cannot run with both asan and valgrind")

    # Change into the build directory
    os.chdir(os.path.join(b.project_dir, "..", "build"))

    # Add 'bin/` to the command (first argument)
    args[0] = os.path.join("bin", args[0])

    # Get current environment
    env = os.environ

    # Add necessary ASAN environment variables
    if use_asan:
        env.update({"ASAN_OPTIONS": "log_path=/home/nubots/NUbots/asan.log"})

    if use_gdb:
        cmd = [
            "gdb",
            "-ex",
            "'set logging redirect on'",
            "-ex",
            "'set logging file /home/nubots/build/gdb.log'",
            "-ex",
            "r",
            "--args",
        ]
    elif use_valgrind:
        cmd = [
            "valgrind",
            "--log-file=/home/nubots/build/valgrind.log",
            "--show-error-list=yes",
            "--leak-check=full",
            "--show-leak-kinds=all",
        ]
    else:
        cmd = []

    # Run the command
    pty = WrapPty()
    exit(pty.spawn(cmd + args, env))
