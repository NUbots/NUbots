#!/usr/bin/env python3

import os
import subprocess

from termcolor import cprint

import b


def register(command):
    command.help = "Build and run images for use with Webots simulation environment"

    command.add_argument(
        "-b",
        "--build",
        dest="isBuild",
        action="store_true",
        default=False,
        help="Build a docker image for use with Webots simulation environment",
    )

    command.add_argument(
        "-r",
        "--run",
        dest="isRun",
        action="store_true",
        default=False,
        help="Run a container",
    )

    command.add_argument(
        "-c",
        "--clean",
        dest="isClean",
        action="store_true",
        default=False,
        help="Cleans config (./b configure --clean)",
    )

    command.add_argument(
        "-p",
        "--push",
        dest="isPush",
        action="store_true",
        default=False,
        help="Pushes image to Docker Hub",
    )


def run(isBuild, isRun, isClean, isPush, **kwargs):
    if isBuild or isRun or isPush:

        # Build docker image
        if isBuild:
            # Set target
            err = subprocess.run(["./b", "target", "robocup2021"]).returncode
            if err != 0:
                cprint("returned exit code {}".format(err), "red", attrs=["bold"])
                exit(err)

            if isClean:
                # Clean config
                err = subprocess.run(["./b", "configure", "--clean"]).returncode
                if err != 0:
                    cprint("returned exit code {}".format(err), "red", attrs=["bold"])
                    exit(err)

            # Set build config
            err = subprocess.run(
                ["./b", "configure", "--", "-DCMAKE_BUILD_TYPE=Release", "--", "-DROLE_robocup2021=ON"]
            ).returncode
            if err != 0:
                cprint("returned exit code {}".format(err), "red", attrs=["bold"])
                exit(err)

            # Build code
            err = subprocess.run(["./b", "build", "robocup2021"]).returncode
            if err != 0:
                cprint("returned exit code {}".format(err), "red", attrs=["bold"])
                exit(err)

            # Copy code out of container?

            # Build image!

        # Run docker container
        if isRun:
            # Check image exists

            # Run!
            pass

        # Push image to Docker Hub
        if isPush:
            # Check image exists

            # Push
            pass

    else:
        # Probably a better way to call help when no sub commands are given?
        os.chdir(b.project_dir)
        tool_name = os.path.basename(__file__)[:-3]
        exit(subprocess.run(["./b", tool_name, "--help"]).returncode)
