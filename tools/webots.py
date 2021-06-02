#!/usr/bin/env python3

import glob
import os
import subprocess
from pathlib import Path

from termcolor import cprint

import b


def register(command):
    command.help = "Build and run images for use with Webots"
    subparsers = command.add_subparsers(help="sub-command help", dest="sub_command")

    build_subcommand = subparsers.add_parser("build", "Build the docker image for use with Webots")
    run_subcommand.add_argument("roles", nargs="+", help="The roles to build for the image")

    run_subcommand = subparsers.add_parser("run", "Run the simulation docker image for use with Webots")
    run_subcommand.add_argument("role", help="The role to run")


def get_cmake_flags(roles_to_build):
    roles_dir = os.path.join(b.project_dir, "roles")
    roles_glob = os.path.join(roles_dir, "*.role")

    available_roles = [Path(role_path).stem for role_path in glob.glob(roles_glob)]

    # Ensure that all the roles requested are available
    for role in roles_to_build:
        if role not in available_roles:
            print(f"role '{role}' not found")
            sys.exit(1)

    role_flags = [f"-DROLE_{role}=ON" for role in available_roles if role in roles_to_build] + [
        f"-DROLE_{role}=OFF" for role in available_roles if role not in roles_to_build
    ]

    return ["-DCMAKE_BUILD_TYPE=Release"] + role_flags


def exec_build(roles):
    # Set target and build the base docker image
    exit_code = subprocess.run(["./b", "target", "generic"]).returncode
    if exit_code != 0:
        cprint("unable to set target, exit code {}".format(exit_code), "red", attrs=["bold"])
        sys.exit(exit_code)

    # Configure the build
    exit_code = subprocess.run(["./b", "configure", "--"] + get_cmake_flags(roles)).returncode
    if exit_code != 0:
        cprint("unable to configure build, exit code {}".format(exit_code), "red", attrs=["bold"])
        sys.exit(exit_code)

    # Build the code
    exit_code = subprocess.run(["./b", "build"]).returncode
    if exit_code != 0:
        cprint("unable to build code, exit code {}".format(exit_code), "red", attrs=["bold"])
        sys.exit(exit_code)

    # Copy compiled binaries and toolchain files out of the build volume by `./b install`ing to a local folder
    exit_code = subprocess.run(["./b", "install", "local", "-co", "-t"]).returncode
    if exit_code != 0:
        cprint(
            "unable to install to local directory, exit code {}".format(exit_code),
            "red",
            attrs=["bold"],
        )
        sys.exit(exit_code)

    # Change into the docker folder
    os.chdir(os.path.join(b.project_dir, "docker"))

    # Build the image!
    exit_code = subprocess.run(
        ["docker", "build", "-t", "nugus_sim:robocup", "-f", "./nugus_sim.Dockerfile", "."]
    ).returncode
    if exit_code != 0:
        cprint(
            "unable to build nugus_sim docker image, exit code {}".format(exit_code),
            "red",
            attrs=["bold"],
        )
        sys.exit(exit_code)

    print("built and tagged docker image nugus_sim:robocup")


def exec_run(role):
    docker_run_command = [
        "docker",
        "container",
        "run",
        "--rm",
        "--network=host",
        "-e",
        "ROBOCUP_ROBOT_ID=1",
        "-e",
        "ROBOCUP_TEAM_COLOR=blue",
        "-e",
        "ROBOCUP_SIMULATOR_ADDR=127.0.0.1:10020",
        "nugus_sim:robocup",
        role,
    ]

    subprocess.run(docker_run_command)


def run(sub_command, roles=None, role=None, **kwargs):
    if sub_command == "build":
        exec_build(roles)
    elif sub_command == "run":  # For testing
        exec_run(role)
    else:
        print(f"invalid sub command: '{sub_command}'")
