#!/usr/bin/env python3

import os

from termcolor import cprint

import b
from dockerise import WrapPty, run_on_docker


@run_on_docker
def register(command):
    # Install help
    command.help = "Run a built binary within the local docker container"

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
def run(args, use_gdb, use_valgrind, **kwargs):

    # Check to see if ASan was enabled
    use_asan = b.cmake_cache["USE_ASAN"] == "ON"

    # Check mutual exclusive status of command line options
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
        cprint("WARN: ASan is enabled. Set USE_ASAN to OFF and rebuild to disable.", "red", attrs=["bold"])

        # Append log_path option if other options have been set
        if "ASAN_OPTIONS" in env:
            # Only append log_path if it hasn't already been set
            if "log_path" not in env["ASAN_OPTIONS"]:
                env["ASAN_OPTIONS"] = "{}:log_path=/home/nubots/NUbots/asan.log".format(env["ASAN_OPTIONS"])
        else:
            env.update({"ASAN_OPTIONS": "log_path=/home/nubots/NUbots/asan.log"})

    # Start role with GDB
    if use_gdb:
        # Setup the gdb command
        cmd = ["gdb"]

        # Add breakpoint to stop asan before it reports an error
        if use_asan:
            cmd.extend(["-ex", "set breakpoint pending on"])
            cmd.extend(["-ex", "br __asan::ReportGenericError"])

        # Start the role
        cmd.extend(["-ex", "r", "--args"])

    # Start role with valgrind
    elif use_valgrind:
        cmd = [
            "valgrind",
            "--log-file=/home/nubots/NUbots/valgrind.log",
            "--show-error-list=yes",
            "--leak-check=full",
            "--show-leak-kinds=all",
        ]
    else:
        cmd = []

    # Run the command
    pty = WrapPty()
    exit(pty.spawn(cmd + args, env))
