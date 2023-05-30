#!/usr/bin/env python3

import os
import subprocess

from termcolor import cprint

import b
from utility.dockerise import defaults, platform


def register(command):
    command.description = "Select the default platform to use for docker commands"

    command.add_argument(
        "target", nargs="?", choices=platform.list(), help="the platform to select as the default platform"
    )
    command.add_argument("--username", default=defaults.local_user, help="the username to tag the completed image as")
    command.add_argument("--uid", default=os.getuid(), help="the uid to set when creating the image")
    command.add_argument(
        "-r", "--reset", default=False, action="store_true", dest="reset", help="Reset buildx instance"
    )


def run(target, username, uid, reset, **kwargs):
    if target is None:
        target = platform.selected(defaults.image, username)
        print(f"Currently selected platform is {target}")
    else:
        # Ensure the platform image is built
        platform.build(defaults.image, target, username, uid, reset)

        # Tag the built platform image as the selected image
        err = subprocess.call(
            [
                "docker",
                "image",
                "tag",
                defaults.image_name(target, username=username),
                defaults.image_name("selected", username=username),
            ]
        )
        if err != 0:
            cprint("docker image tag returned exit code {}".format(err), "red", attrs=["bold"])
            exit(err)
