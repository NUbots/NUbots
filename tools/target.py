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


def run(target, **kwargs):

    if target is None:
        target = platform.selected(defaults.image)
        print("Currently selected platform is {}".format(target))
    else:
        # Ensure the platform image is built
        platform.build(defaults.image, target)

        # Tag the built platform image is the selected image
        tag = "{}:{}".format(defaults.image, target)
        err = subprocess.run(["docker", "image", "tag", tag, "{}:selected".format(defaults.image)]).returncode
        if err != 0:
            cprint("docker image tag returned exit code {}".format(err), "red", attrs=["bold"])
            exit(err)
