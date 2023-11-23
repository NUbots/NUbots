#!/usr/bin/env python3
#
# MIT License
#
# Copyright (c) 2019 NUbots
#
# This file is part of the NUbots codebase.
# See https://github.com/NUbots/NUbots for further info.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#

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
