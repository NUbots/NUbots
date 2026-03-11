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

    mode = command.add_mutually_exclusive_group()
    mode.add_argument(
        "--pull",
        default=False,
        action="store_true",
        help="Pull a prebuilt platform image from the registry and select it without building",
    )
    mode.add_argument(
        "--use-existing",
        default=False,
        action="store_true",
        dest="use_existing",
        help="Use an existing local image (or an already-pulled upstream tag) and select it without building",
    )


def _image_exists(tag: str) -> bool:
    return subprocess.run(["docker", "image", "inspect", tag], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0


def run(target, username, uid, reset, pull, use_existing, **kwargs):
    if target is None:
        target = platform.selected(defaults.image, username)
        print(f"Currently selected platform is {target}")
    else:
        upstream_tag = f"{defaults.image_user}/{defaults.image}:{target}"
        local_target_tag = defaults.image_name(target, username=username)
        local_selected_tag = defaults.image_name("selected", username=username)

        if pull:
            err = subprocess.call(["docker", "pull", upstream_tag])
            if err != 0:
                cprint(f"docker pull returned exit code {err}", "red", attrs=["bold"])
                exit(err)

            err = subprocess.call(["docker", "image", "tag", upstream_tag, local_target_tag])
            if err != 0:
                cprint(f"docker image tag returned exit code {err}", "red", attrs=["bold"])
                exit(err)

        elif use_existing:
            source_tag = None
            if _image_exists(local_target_tag):
                source_tag = local_target_tag
            elif _image_exists(upstream_tag):
                source_tag = upstream_tag
            else:
                cprint(
                    f"No local image found for {local_target_tag} or {upstream_tag}.\n"
                    f"Try running `./b target {target}` to build, or `./b target {target} --pull`.",
                    "red",
                    attrs=["bold"],
                )
                exit(1)

            # Ensure we have the local tag in place (even if the source is already local)
            if source_tag != local_target_tag:
                err = subprocess.call(["docker", "image", "tag", source_tag, local_target_tag])
                if err != 0:
                    cprint(f"docker image tag returned exit code {err}", "red", attrs=["bold"])
                    exit(err)

        else:
            # Ensure the platform image is built
            platform.build(defaults.image, target, username, uid, reset)

        # Tag the built platform image as the selected image
        err = subprocess.call(
            [
                "docker",
                "image",
                "tag",
                local_target_tag,
                local_selected_tag,
            ]
        )
        if err != 0:
            cprint("docker image tag returned exit code {}".format(err), "red", attrs=["bold"])
            exit(err)
