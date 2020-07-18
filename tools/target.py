#!/usr/bin/env python3

import os

from termcolor import cprint

import b

from dockerise import WrapPty, build_platform, get_selected_platform, platforms, repository

import docker


def register(command):
    command.help = "Select the default platform to use for docker commands"

    command.add_argument(
        "platform",
        metavar="platform",
        nargs="?",
        choices=platforms(),
        help="the platform to select as the default platform",
    )


def run(platform, **kwargs):

    if platform is None:
        platform = get_selected_platform()
        print("Currently selected platform is {}".format(platform))
    else:
        # Ensure the platform image is built
        build_platform(platform)

        # Tag the built platform image is the selected image
        pty = WrapPty()
        tag = "{}:{}".format(repository, platform)
        err = pty.spawn(["docker", "image", "tag", tag, "{}:selected".format(repository)])
        if err != 0:
            cprint("docker image tag returned exit code {}".format(err), "red", attrs=["bold"])
            exit(err)
