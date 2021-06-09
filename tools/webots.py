#!/usr/bin/env python3

import glob
import os
import shutil
import subprocess
import sys
import textwrap
from pathlib import Path

from termcolor import cprint

import b

# The docker image details for Robocup
ROBOCUP_IMAGE_NAME = "robocup-vhsc-nubots"  # Provided by the TC and shouldn't be changed
ROBOCUP_IMAGE_TAG = "robocup2021"  # Submitted in our team_config.json, shouldn't be changed here unless changed there
ROBOCUP_IMAGE_REGISTRY = "079967072104.dkr.ecr.us-east-2.amazonaws.com/robocup-vhsc-nubots"  # Provided by the TC


def register(command):
    command.help = "Build and run images for use with Webots"
    subparsers = command.add_subparsers(help="sub-command help", dest="sub_command")

    build_subcommand = subparsers.add_parser("build", help="Build the docker image for use with Webots")
    build_subcommand.add_argument("roles", nargs="+", help="The roles to build for the image")

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
    print("Setting target 'generic'...")
    exit_code = subprocess.run(["./b", "target", "generic"]).returncode
    if exit_code != 0:
        cprint("unable to set target, exit code {}".format(exit_code), "red", attrs=["bold"])
        sys.exit(exit_code)

    print("Configuring build...")
    configure_command = ["./b", "configure", "--"] + get_cmake_flags(roles)
    exit_code = subprocess.run(configure_command).returncode
    if exit_code != 0:
        cprint("unable to configure build, exit code {}".format(exit_code), "red", attrs=["bold"])
        sys.exit(exit_code)

    print("Building code...")
    exit_code = subprocess.run(["./b", "build"]).returncode
    if exit_code != 0:
        cprint("unable to build code, exit code {}".format(exit_code), "red", attrs=["bold"])
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
            "unable to install to local directory, exit code {}".format(exit_code),
            "red",
            attrs=["bold"],
        )
        sys.exit(exit_code)

    # Change into the docker folder
    os.chdir(os.path.join(b.project_dir, "docker"))

    print(f"Building the docker image {ROBOCUP_IMAGE_NAME}:{ROBOCUP_IMAGE_TAG}...")

    # Build the image!
    exit_code = subprocess.run(
        ["docker", "build", "-t", f"{ROBOCUP_IMAGE_NAME}:{ROBOCUP_IMAGE_TAG}", "-f", "./nugus_sim.Dockerfile", "."]
    ).returncode
    if exit_code != 0:
        cprint(
            "unable to build docker image, exit code {}".format(exit_code),
            "red",
            attrs=["bold"],
        )
        sys.exit(exit_code)

    # Clean up
    shutil.rmtree(local_binaries_dir)
    shutil.rmtree(local_toolchain_dir)


def exec_run(role):
    robocup_logs_dir = os.path.join(b.project_dir, "robocup-logs")

    # Ensure the logs directory exists for binding into the container
    os.makedirs(robocup_logs_dir, exist_ok=True)

    docker_run_command = [
        "docker",
        "container",
        "run",
        "--rm",
        "--network=host",
        "--mount",
        f"type=bind,source={robocup_logs_dir},target=/robocup-logs",
        "-e",
        "ROBOCUP_ROBOT_ID=1",
        "-e",
        "ROBOCUP_TEAM_COLOR=red",
        "-e",
        "ROBOCUP_SIMULATOR_ADDR=127.0.0.1:10001",
        f"{ROBOCUP_IMAGE_NAME}:{ROBOCUP_IMAGE_TAG}",
        role,
    ]

    subprocess.run(docker_run_command)


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
        sys.exit(1)

    exit_code = subprocess.run(
        [
            "docker",
            "push",
            f"{ROBOCUP_IMAGE_REGISTRY}:{ROBOCUP_IMAGE_TAG}",
        ]
    ).returncode

    if exit_code != 0:
        print(f"unable to push image, exit code {exit_code}")
        sys.exit(1)


def run(sub_command, roles=None, role=None, **kwargs):
    if sub_command == "build":
        exec_build(roles)
    elif sub_command == "push":
        exec_push()
    elif sub_command == "run":  # For testing
        exec_run(role)
    else:
        print(f"invalid sub command: '{sub_command}'")
