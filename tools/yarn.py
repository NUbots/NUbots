#!/usr/bin/env python3
#
# MIT License
#
# Copyright (c) 2021 NUbots
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

import argparse
import os
import re
import subprocess

import b
from utility.dockerise import run_on_docker
from utility.shell import WrapPty
from utility.yarn import get_nusight_ports


def _is_linux():
    kernel_release = subprocess.check_output(["uname", "-a"]).decode("utf-8").strip()
    is_macos_or_wsl = re.search("Darwin|Microsoft", kernel_release, re.IGNORECASE)
    return not is_macos_or_wsl


def _get_ports():
    parser = argparse.ArgumentParser(usage=argparse.SUPPRESS, add_help=False)
    parser.add_argument("b_command", nargs="?")
    parser.add_argument("yarn_command", nargs="?")
    parser.add_argument("--network", dest="network")

    try:
        args, _ = parser.parse_known_intermixed_args()
    except:
        return []

    if args.b_command != "yarn":
        return []

    if args.network == "host" or (args.network is None and _is_linux()):
        return []

    return get_nusight_ports(args.yarn_command)


@run_on_docker
def register(command):
    command.description = "Run yarn in the docker container"
    command.add_argument("args", nargs="...", help="the arguments to pass through to yarn")


@run_on_docker(ports=_get_ports())
def run(args, **kwargs):
    pty = WrapPty()

    # Change into the NUsight directory
    os.chdir(os.path.join(b.project_dir, "nusight2"))

    # Run the yarn command
    exit(pty.spawn(["yarn", *args]))
