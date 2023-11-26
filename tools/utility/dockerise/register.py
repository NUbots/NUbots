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

from . import defaults


def register(func, image, **kwargs):
    def _register(command):

        # Only if we are using the internal image does it make sense to provide things like rebuild and clean
        if defaults.internalise_image(image)[0]:
            command.add_argument(
                "--clean", action="store_true", help="delete and recreate all docker volumes (build directories)"
            )

        command.add_argument("--rebuild", action="store_true", help="rebuild the docker image checking for updates")
        command.add_argument(
            "--environment",
            dest="environment",
            help="Run the container with extra environment variables. Set variables as VAR1=VALUE1,VAR2=VALUE2",
        )
        command.add_argument("--network", dest="network", help="Run the container on the specified docker network")
        command.add_argument("--mount", default=[], action="append", help="--mount commands to pass to docker run")
        command.add_argument("--volume", default=[], action="append", help="--volume commands to pass to docker run")
        command.add_argument("--gpus", help="--gpus commands to pass to docker run")

        func(command)

    return _register
