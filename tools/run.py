#!/usr/bin/env python3

import os
from glob import glob

from termcolor import cprint

import b
from utility.dockerise import run_on_docker
from utility.shell import WrapPty


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

    # Find all role files
    fnames = glob(os.path.join(b.project_dir, "roles", "**", "*.role"), recursive=True)
    # Strip everything from file paths except role name and subdirectory in roles/
    roles = [os.path.splitext(f.replace(os.path.join(b.project_dir, "roles", ""), ""))[0] for f in fnames]

    command.add_argument("role", help="The role to run", choices=roles)
    command.add_argument("args", nargs="*", help="Any arguments that should be used for the execution")


@run_on_docker
def run(role, args, use_gdb, use_valgrind, **kwargs):

    # Check to see if ASan was enabled
    use_asan = b.cmake_cache["USE_ASAN"] == "ON"

    # Check mutual exclusive status of command line options
    if use_gdb and use_valgrind:
        raise Exception("Cannot run with both gdb and valgrind")
    if use_asan and use_valgrind:
        raise Exception("Cannot run with both asan and valgrind")

    # Change into the build directory
    os.chdir(os.path.join(b.project_dir, "..", "build"))

    # Path to the binary being run starting at bin/
    binary = os.path.join("bin", role)

    # Get current environment
    env = os.environ

    # Make sure the log directory exists
    try:
        os.makedirs(os.path.join(b.project_dir, "log"), exist_ok=True)
    # The file exists, and but it's not a directory
    except FileExistsError:
        # Move the file to a temporary, make the new directory, then move the temporary file into the new directory
        os.rename(os.path.join(b.project_dir, "log"), os.path.join(b.project_dir, "log.tmp"))
        os.makedirs(os.path.join(b.project_dir, "log"))
        os.rename(os.path.join(b.project_dir, "log.tmp"), os.path.join(b.project_dir, "log", "log"))

    # Add necessary ASAN environment variables
    if use_asan:
        cprint("WARN: ASan is enabled. Set USE_ASAN to OFF and rebuild to disable.", "red", attrs=["bold"])

        # Append log_path option if other options have been set
        if "ASAN_OPTIONS" in env:
            # Only append log_path if it hasn't already been set
            if "log_path" not in env["ASAN_OPTIONS"]:
                env["ASAN_OPTIONS"] = "{}:log_path=/home/nubots/NUbots/log/asan.log".format(env["ASAN_OPTIONS"])
        else:
            env.update({"ASAN_OPTIONS": "log_path=/home/nubots/NUbots/log/asan.log"})

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
            "--log-file=/home/nubots/NUbots/log/valgrind.log",
            "--show-error-list=yes",
            "--leak-check=full",
            "--show-leak-kinds=all",
        ]
    else:
        cmd = []

    cmd.append(binary)

    # Run the command
    pty = WrapPty()
    exit(pty.spawn(cmd + args, env))
