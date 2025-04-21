#!/usr/bin/env python3
#
# MIT License
#
# Copyright (c) 2025 NUbots
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

# This tool is used to register the multi command and relevant arguments for use in the dockerise/run.py script.
# The run command here is not used, as the multi command is not run in the docker container.
# It is instead used to run multiple docker containers with the 'run' command.

from utility.dockerise import run_on_docker


@run_on_docker
def register(command):
    # Install help
    command.description = "Edit a file within the local docker container"

    command.add_argument("num_robots", type=int, help="The number of robots to run per team")
    command.add_argument("--single_team", action="store_true", help="Only run one team")
    command.add_argument("args", nargs="+", help="the command and any arguments that should be used for the execution")

@run_on_docker
def run(args, **kwargs):
    return
