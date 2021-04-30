#!/usr/bin/env python3

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
        "-l",
        "--local-build",
        dest="build_local",
        action="store_true",
        default=False,
        help="build the image locally, instead of pulling and tagging it",
    )


def run(target, build_local, **kwargs):

    # Print the current target if no target was selected
    if target is None:
        target = platform.selected(defaults.image)
        print("Currently selected platform is {}".format(target))

    else:
        # If user wants to build it locally, do that
        if build_local:
            platform.build(defaults.image, target)
        # Else, pull the image from dockerhub
        else:
            platform.pull(defaults.image, target)

        # Tag the result image as the selected image
        tag = "{}:{}".format(defaults.image, target)
        print("Tagging", tag, "as {}:selected".format(defaults.image))

        err = subprocess.run(["docker", "image", "tag", tag, "{}:selected".format(defaults.image)]).returncode
        if err != 0:
            cprint("docker image tag returned exit code {}".format(err), "red", attrs=["bold"])
            exit(err)
