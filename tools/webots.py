#!/usr/bin/env python3
#
# MIT License
#
# Copyright (c) 2021 NUbots
#
# This file is part of the NUbots codebase.
# See https://github.com/NUbots/NUbots for further info.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#

import multiprocessing
import os
import random
import shutil
import subprocess
import sys
import textwrap
from datetime import datetime
from glob import glob
from pathlib import Path
from subprocess import DEVNULL

from honcho.manager import Manager
from termcolor import cprint

import b
from utility.dockerise import platform

random.seed(datetime.now())

# The docker image details for Robocup
ROBOCUP_IMAGE_NAME = "robocup-vhsc-nubots"  # Provided by the TC and shouldn't be changed
ROBOCUP_IMAGE_TAG = "robocup"  # Submitted in our team_config.json, shouldn't be changed here unless changed there
ROBOCUP_IMAGE_REGISTRY = "047817357099.dkr.ecr.us-east-2.amazonaws.com/hl-vs-nubots"  # Provided by the TC

# NUbots RoboCup team ID
NUBOTS_TEAM_ID = 12

# Generate 4 digit random int to act as a unique identifier for this game
GAME_IDENTIFIER = str(random.randint(1000, 9999))


def register(command):
    command.help = "Build and run images for use with Webots"
    subparsers = command.add_subparsers(help="sub-command help", dest="sub_command")

    build_subcommand = subparsers.add_parser("build", help="Build the docker image for use with Webots")
    build_subcommand.add_argument("roles", nargs="+", help="The roles to build for the image")
    build_subcommand.add_argument(
        "--target",
        nargs="?",
        choices=platform.list(),
        default="generic",
        help="The platform to compile for",
    )

    build_subcommand.add_argument(
        "--no-clean",
        dest="clean",
        action="store_false",
        default=True,
        help="Disables cleaning the build volume before compiling",
    )

    build_subcommand.add_argument(
        "-j",
        "--jobs",
        dest="jobs",
        action="store",
        help="Dictates the number of jobs to run in parallel",
    )

    push_subcommand = subparsers.add_parser(
        "push",
        help=textwrap.dedent(
            """
            Push the simulation docker image to the TC registry. Configure the aws CLI and login with the TC credentials before running this command.
            See "Uploading Docker Images" in the Robocup "API Specifications for Virtual Soccer Competition" document for details.
        """
        ),
    )

    run_subcommand = subparsers.add_parser("run", help="Run the simulation docker image for use with Webots")
    run_subcommand.add_argument("role", help="The role to run")

    game_subcommand = subparsers.add_parser("game", help="Run a full game")
    game_subcommand.add_argument("role", action="store", help="The role to run")
    game_subcommand.add_argument(
        "-r",
        "--robots",
        type=int,
        action="store",
        dest="num_of_robots",
        required=True,
        help="The number of robot instances to start",
    )
    game_subcommand.add_argument(
        "-a",
        "--sim-address",
        action="store",
        default="127.0.0.1",
        dest="sim_address",
        help="Address that robot instances use to connect to Webots",
    )


def get_cmake_flags(roles_to_build):

    # Find all available roles
    available_roles = glob(os.path.join(b.project_dir, "roles", "**", "*.role"), recursive=True)
    available_roles = [role.split(os.path.join(b.project_dir, "roles/"))[1] for role in available_roles]
    available_roles = [role.split(".")[0] for role in available_roles]

    # Ensure that all the roles requested are available
    for role in roles_to_build:
        if role not in available_roles:
            cprint(f"role '{role}' not found", color="red", attrs=["bold"])
            sys.exit(1)

    role_flags = [f"-DROLE_{role}=ON" for role in available_roles if role in roles_to_build] + [
        f"-DROLE_{role}=OFF" for role in available_roles if role not in roles_to_build
    ]

    return ["-DCMAKE_BUILD_TYPE=Release"] + role_flags


def exec_build(target, roles, clean, jobs):
    # Tags correct image as 'selected' for given target
    print("Setting target '{}'...".format(target))
    exit_code = subprocess.run(["./b", "target", target]).returncode
    if exit_code != 0:
        cprint("unable to set target to '{}', exit code {}".format(target, exit_code), "red", attrs=["bold"])
        sys.exit(exit_code)

    # Cleans build volume to ensure everything gets build for given target
    if clean:
        print("Cleaning build volume...")
        exit_code = subprocess.run(["./b", "configure", "--clean"]).returncode
        if exit_code != 0:
            cprint("unable to clean build volume, exit code {}".format(exit_code), "red", attrs=["bold"])
            sys.exit(exit_code)

    # Sets cmake flags and roles
    print("Configuring build...")
    configure_command = ["./b", "configure", "--"] + get_cmake_flags(roles)
    exit_code = subprocess.run(configure_command).returncode
    if exit_code != 0:
        cprint(f"unable to configure build, exit code {exit_code}", "red", attrs=["bold"])
        sys.exit(exit_code)

    # Compiles code for correct target
    print("Building code...")
    build_command = ["./b", "build"]
    # Check if a -j value has been given
    if jobs:
        build_command.extend(["-j", jobs])
    exit_code = subprocess.run(build_command).returncode
    if exit_code != 0:
        cprint(f"unable to build code, exit code {exit_code}", "red", attrs=["bold"])
        sys.exit(exit_code)

    # The paths to the built binaries and toolchain on the local filesystem
    local_binaries_dir = os.path.join(b.project_dir, "docker", "nugus_sim", "binaries")
    local_toolchain_dir = os.path.join(b.project_dir, "docker", "nugus_sim", "toolchain")

    # Clean the previous built binaries and toolchain files for a fresh install
    if os.path.exists(local_binaries_dir):
        shutil.rmtree(local_binaries_dir)
    if os.path.exists(local_toolchain_dir):
        shutil.rmtree(local_toolchain_dir)

    # Where to put the built binaries and toolchain on the docker build container filesystem
    # Note that the path below is in the build docker container, since that is where `./b install` runs
    docker_install_dir = os.path.join("/home", "nubots", "NUbots", "docker", "nugus_sim")

    print(f"Installing built binaries and toolchain to {docker_install_dir}...")
    exit_code = subprocess.run(["./b", "install", docker_install_dir, "--local", "-co", "-t"]).returncode
    if exit_code != 0:
        cprint(
            f"unable to install to local directory, exit code {exit_code}",
            color="red",
            attrs=["bold"],
        )
        sys.exit(exit_code)

    # Change into the docker folder
    os.chdir(os.path.join(b.project_dir, "docker"))

    print(f"Building the docker image {ROBOCUP_IMAGE_NAME}:{ROBOCUP_IMAGE_TAG}...")

    # Build the image!
    exit_code = subprocess.run(
        ["docker", "build", "-t", f"{ROBOCUP_IMAGE_NAME}:{ROBOCUP_IMAGE_TAG}", "-f", "./Dockerfile.nugus_sim", "."]
    ).returncode
    if exit_code != 0:
        cprint(
            f"unable to build docker image, exit code {exit_code}",
            color="red",
            attrs=["bold"],
        )
        sys.exit(exit_code)

    # Change back into project dir
    os.chdir(b.project_dir)

    # Set selected image back to 'generic'
    print("Setting back to target 'generic'...")
    exit_code = subprocess.run(["./b", "target", "generic"]).returncode
    if exit_code != 0:
        cprint("unable to set target to 'generic', exit code {}".format(exit_code), "red", attrs=["bold"])
        sys.exit(exit_code)

    # Clean up
    shutil.rmtree(local_binaries_dir)
    shutil.rmtree(local_toolchain_dir)


def exec_run(role, num_of_robots=1, sim_address="127.0.0.1"):
    # Check that image has been built
    exit_code = subprocess.run(
        ["docker", "image", "inspect", f"{ROBOCUP_IMAGE_NAME}:{ROBOCUP_IMAGE_TAG}"], stderr=DEVNULL, stdout=DEVNULL
    ).returncode
    if exit_code != 0:
        cprint(
            f"Image '{ROBOCUP_IMAGE_NAME}:{ROBOCUP_IMAGE_TAG}' not found, please run `./b webots build` to build it",
            "red",
            attrs=["bold"],
        )
        exit(exit_code)

    robocup_logs_dir = os.path.join(b.project_dir, "robocup-logs")

    # Ensure the logs directory exists for binding into the container
    os.makedirs(robocup_logs_dir, exist_ok=True)

    process_manager = Manager()

    # Override honcho.manager.terminate() method, this is called by its signal handler on CTRL+C
    process_manager._killall = exec_stop

    # Add all robot run commands to process_manager
    for i in range(1, num_of_robots + 1):

        # Set team_id, robot_color and port_num based on the number of instances being started
        if i <= num_of_robots // 2:
            team_id = NUBOTS_TEAM_ID
            robot_color = "red"
            port_num = 10000 + i
        else:
            team_id = NUBOTS_TEAM_ID + 1
            robot_color = "blue"
            port_num = 10020 + i - num_of_robots // 2

        docker_run_command = [
            "docker",
            "container",
            "run",
            "--rm",
            "--network=host",
            f"--name={GAME_IDENTIFIER + '_Robot' + str(i)}",
            "--mount",
            f"type=bind,source={robocup_logs_dir},target=/robocup-logs",
            "-e",
            f"ROBOCUP_ROBOT_ID={i}",
            "-e",
            f"ROBOCUP_TEAM_ID={team_id}",
            "-e",
            f"ROBOCUP_TEAM_COLOR={robot_color}",
            "-e",
            f"ROBOCUP_SIMULATOR_ADDR={sim_address}:{port_num}",
            f"{ROBOCUP_IMAGE_NAME}:{ROBOCUP_IMAGE_TAG}",
            role,
        ]
        process_manager.add_process(f"Robot_{i} ({robot_color}):", " ".join(docker_run_command))

    # Start the containers
    process_manager.loop()

    sys.exit(process_manager.returncode)


# Signal handler function to kill containers on CTRL+C
def exec_stop():
    # Get all container ID's for containers running running this game
    container_ids = (
        subprocess.check_output(
            [
                "docker",
                "ps",
                "-a",
                "-q",
                "--filter",
                f"name={GAME_IDENTIFIER}_Robot",
                "--format={{.ID}}",
            ]
        )
        .strip()
        .decode("ascii")
        .split()
    )

    cprint(
        f"Stoping ALL '{GAME_IDENTIFIER}_Robotx' containers...",
        color="red",
        attrs=["bold"],
    )
    exit_code = subprocess.run(
        ["docker", "container", "rm", "-f"] + container_ids, stderr=DEVNULL, stdout=DEVNULL
    ).returncode

    sys.exit(exit_code)


def exec_push():
    exit_code = subprocess.run(
        [
            "docker",
            "tag",
            f"{ROBOCUP_IMAGE_NAME}:{ROBOCUP_IMAGE_TAG}",
            f"{ROBOCUP_IMAGE_REGISTRY}:{ROBOCUP_IMAGE_TAG}",
        ]
    ).returncode

    if exit_code != 0:
        print(f"unable to tag image, exit code {exit_code}")
        sys.exit(exit_code)

    exit_code = subprocess.run(
        [
            "docker",
            "push",
            f"{ROBOCUP_IMAGE_REGISTRY}:{ROBOCUP_IMAGE_TAG}",
        ]
    ).returncode

    if exit_code != 0:
        print(f"unable to push image, exit code {exit_code}")
        sys.exit(exit_code)


def run(
    sub_command,
    jobs=None,
    target="generic",
    sim_address="127.0.0.1",
    clean=False,
    roles=None,
    role=None,
    num_of_robots=1,
    **kwargs,
):
    if sub_command == "build":
        exec_build(target, roles, clean, jobs)
    elif sub_command == "push":
        exec_push()
    elif sub_command == "run":  # For testing docker image
        exec_run(role)
    elif sub_command == "game":  # For running a full game
        exec_run(role, num_of_robots, sim_address)
    else:
        print(f"invalid sub command: '{sub_command}'")
        sys.exit(1)
