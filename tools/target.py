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
        default=False,
        help="build the image locally, instead of pulling and tagging it",
    )

    command.add_argument(
        "-n",
        "--no-pull",
        dest="should_pull",
        action="store_false",
        default=True,
        help="disable pulling the image from dockerhub",
    )


def run(target, build, should_pull, **kwargs):

    # Print the current target if no target was selected
    if target is None:
        target = platform.selected(defaults.image)
        print("Currently selected platform is {}".format(target))

    else:
        # We check if a build is necessary by diffing the docker folder with master
        changed = (
            subprocess.run(
                ["git", "diff", "--exit-code", "origin/master", os.path.join(b.project_dir, "docker")],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            ).returncode
            != 0
        )

        # If there are changes in tracked files in the docker folder when comparing with master,
        # we want to build the image and we don't want to pull the image
        if changed:
            build = True
            should_pull = False

        if should_pull:
            # Pull the image from dockerhub
            platform.pull(defaults.image, target)

        # If user wants or needs to build it locally, do that
        if build:
            platform.build(defaults.image, target)

        # Tag the result image as the selected image
        tag = "{}:{}".format(defaults.image, target)
        print("Tagging", tag, "as {}:selected".format(defaults.image))

        err = subprocess.run(["docker", "image", "tag", tag, "{}:selected".format(defaults.image)]).returncode
        if err != 0:
            cprint("docker image tag returned exit code {}".format(err), "red", attrs=["bold"])
            exit(err)
