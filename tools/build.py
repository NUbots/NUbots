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
import shutil
import subprocess

from configure import run as configure
from termcolor import cprint

import b
from utility.dockerise import run_on_docker


@run_on_docker
def register(command):
    # Install help
    command.description = "build the codebase"

    command.add_argument("args", nargs="...", help="the arguments to pass through to ninja")
    command.add_argument("-j", help="number of jobs to spawn")
    command.add_argument("-m", "--purge-messages", action="store_true", help="Delete all compiled proto messages files")


@run_on_docker
def run(j, args, purge_messages, **kwargs):
    # Change into the build directory
    os.chdir(os.path.join(b.project_dir, "..", "build"))

    # Run cmake if we don't have a ninja build file
    if not os.path.isfile("build.ninja"):
        configure(interactive=False, set_roles=[], unset_roles=[], args=[])

    # Purge compiled messages
    if purge_messages:
        cprint("Purging all compiled messages from build folder", "red", attrs=["bold"])
        shutil.rmtree(os.path.join("nuclear", "message"))

    # To pass arguments to the ninja command you put them after "--"
    # but "--"  isn't a valid argument for ninja, so we remove it here
    if "--" in args:
        args.remove("--")

    command = ["ninja", *args]

    if j:
        command.insert(1, "-j{}".format(j))

    # Return the exit code of ninja
    exit(subprocess.call(command))
