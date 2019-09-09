#!/usr/bin/env python3

import os
import re
import b
import docker
import dockerise


def register(command):
    command.help = "Select the default platform to use for docker commands"

    # Get the possible platforms
    p = re.compile("generate_([a-z0-9]+)_toolchain.py")
    platforms = [
        m.group(1)
        for m in [p.match(s) for s in os.listdir(os.path.join(b.project_dir, "docker", "usr", "local", "toolchain"))]
        if m is not None
    ]

    command.add_argument(
        "platform",
        metavar="platform",
        nargs="?",
        choices=platforms,
        help="the platform to select as the default platform",
    )


def run(platform, **kwargs):

    if platform is None:
        platform = dockerise.get_selected_platform()
        print("Currently selected platform is {}".format(platform))
    else:
        tag = "{}:{}".format(dockerise.repository, platform)

        try:
            img = dockerise.client.images.get(tag)
        except docker.errors.ImageNotFound:
            dockerise.build_platform(platform)
            img = dockerise.client.images.get(tag)

        img.tag(dockerise.repository, "selected")
