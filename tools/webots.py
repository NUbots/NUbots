#!/usr/bin/env python3

import os
import subprocess

from termcolor import cprint

import b


def register(command):

    command.help = "Build and run images for use with Webots simulation environment"
    subparsers = command.add_subparsers(help="sub-command help", dest="sub_command")

    command_build = subparsers.add_parser("build")
    command_run = subparsers.add_parser("run")

    command_build.add_argument(
        "-c",
        "--clean",
        dest="clean",
        action="store_true",
        default=False,
        help="Cleans config (./b configure --clean)",
    )

    command_build.add_argument(
        "-p",
        "--push",
        dest="push",
        action="store_true",
        default=False,
        help="Pushes image to Docker Hub",
    )

    command_run.add_argument("role", nargs=1, help="Role to run")


def run(sub_command, role=None, **kwargs):
    if sub_command == "build":

        # # Set target
        # err = subprocess.run(["./b", "target", "robocup2021"]).returncode
        # if err != 0:
        #     cprint("returned exit code {}".format(err), "red", attrs=["bold"])
        #     exit(err)

        # Remove build volume
        if kwargs["clean"] == True:
            # Clean config
            err = subprocess.run(["./b", "configure", "--clean"]).returncode
            if err != 0:
                cprint("returned exit code {}".format(err), "red", attrs=["bold"])
                exit(err)

        # Set build config
        err = subprocess.run(
            ["./b", "configure", "--", "-DCMAKE_BUILD_TYPE=Release", "--", "-DROLE_webots=ON"]
        ).returncode
        if err != 0:
            cprint("returned exit code {}".format(err), "red", attrs=["bold"])
            exit(err)

        # Build code
        err = subprocess.run(["./b", "build"]).returncode
        if err != 0:
            cprint("returned exit code {}".format(err), "red", attrs=["bold"])
            exit(err)

        # Copy compiled binaries and runtime dependancies out of build volume
        err = subprocess.run(["./b", "install", "local", "-t"]).returncode
        if err != 0:
            cprint("returned exit code {}".format(err), "red", attrs=["bold"])
            exit(err)

        # Build image!
        os.chdir(b.project_dir + "/docker")
        err = subprocess.run(
            ["docker", "build", "-t", "nugus_sim:robocup", "-f", "./nugus_sim_Dockerfile", "."]
        ).returncode
        if err != 0:
            cprint("returned exit code {}".format(err), "red", attrs=["bold"])
            exit(err)

    # Just for testing atm
    elif sub_command == "run":
        docker_run_command = [
            "docker",
            "container",
            "run",
            "--rm",
            "-e",
            "ROBOCUP_ROBOT_ID=1",
            "-e",
            "ROBOCUP_TEAM_COLOR=1",
            "-e",
            "ROBOCUP_SIMULATOR_ADDR=127.0.0.1:10020",
            "nugus_sim:robocup",
        ]
        docker_run_command.append(role[0])
        subprocess.run(docker_run_command)

    else:
        # Probably a better way to call help when no sub commands are given?
        os.chdir(b.project_dir)
        tool_name = os.path.basename(__file__)[:-3]
        exit(subprocess.run(["./b", tool_name, "--help"]).returncode)
