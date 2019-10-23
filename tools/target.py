#!/usr/bin/env python3

import os

import b
import docker
import dockerise


def register(command):
    command.help = "Select the default platform to use for docker commands"

    command.add_argument(
        "platform",
        metavar="platform",
        nargs="?",
        choices=dockerise.platforms(),
        help="the platform to select as the default platform",
    )


def run(platform, **kwargs):

    if platform is None:
        platform = dockerise.get_selected_platform()
        print("Currently selected platform is {}".format(platform))
    else:
        tag = "{}:{}".format(dockerise.repository, platform)
        dockerise.build_platform(platform)
        img = dockerise.client.images.get(tag)

        img.tag(dockerise.repository, "selected")
