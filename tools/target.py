#!/usr/bin/env python3

import os
import subprocess

from termcolor import cprint

import b
from utility.dockerise import defaults, platform


def register(command):
    command.help = "Select the default platform to use for docker commands"

    command.add_argument(
        "target", nargs="?", choices=platform.list(), help="the platform to select as the default platform"
    )

    command.add_argument(
        "-b",
        "--build",
        dest="build",
        action="store_true",
        help="build the image locally, instead of pulling and tagging it",
    )

    command.add_argument(
        "-n",
        "--no-pull",
        dest="pull",
        action="store_false",
        help="disable pulling the image from dockerhub",
    )


def run(target, build, pull, **kwargs):

    # Print the current target if no target was selected
    if target is None:
        target = platform.selected(defaults.image)
        print("Currently selected platform is {}".format(target))

    else:
        # We check if a build is necessary by diffing the docker folder with main
        changed = (
            subprocess.run(
                ["git", "diff", "--exit-code", "origin/main", os.path.join(b.project_dir, "docker")],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            ).returncode
            != 0
        )

        if pull:
            # Pull the image from dockerhub
            platform.pull(defaults.image, target)

        # If user wants or needs to build it locally, do that unless the build context is the same as origin/main
        if build:
            if not changed:
                cprint(
                    "There aren't any changes to be made to the docker image. "
                    "Skipping build, and using tagged image instead",
                    "red",
                    attrs=["bold"],
                )
            else:
                platform.build(defaults.image, target)
        # If the user hasn't selected build but they have changed the docker directory, then we warn them that the image
        # isn't going to be built, even though the dirty diff implies they might want it to be built
        else:
            if changed:
                cprint(
                    "Docker build context differs from origin/main, and build not selected. "
                    "This is likely an error.",
                    "red",
                    attrs=["bold"],
                )

        # Tag the result image as the selected image
        tag = "{}:{}".format(defaults.image, target)
        print("Tagging", tag, "as {}:selected".format(defaults.image))

        err = subprocess.run(["docker", "image", "tag", tag, "{}:selected".format(defaults.image)]).returncode
        if err != 0:
            cprint("docker image tag returned exit code {}".format(err), "red", attrs=["bold"])
            exit(err)
