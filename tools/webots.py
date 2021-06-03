#!/usr/bin/env python3

import glob
import os
import shutil
import subprocess
import sys
from pathlib import Path

from termcolor import cprint

import b


def register(command):
    command.help = "Build and run images for use with Webots"
    subparsers = command.add_subparsers(help="sub-command help", dest="sub_command")

    build_subcommand = subparsers.add_parser("build", help="Build the docker image for use with Webots")
    build_subcommand.add_argument("roles", nargs="+", help="The roles to build for the image")

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
    print(" ".join(configure_command))

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

    # The docker image details
    # The image name is given by the TC and shouldn't be changed
    # The image tag is submitted in our team_config.json and shouldn't be changed
    image_name = "robocup-vhsc-nubots"
    image_tag = "robocup2021"

    print(f"Building the docker image {image_name}:{image_tag}...")

    # Build the image!
    exit_code = subprocess.run(
        ["docker", "build", "-t", f"{image_name}:{image_tag}", "-f", "./nugus_sim.Dockerfile", "."]
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
        "robocup-vhsc-nubots:robocup2021",
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
