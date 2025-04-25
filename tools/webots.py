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

import os
import re
import shutil
import subprocess
import sys
import textwrap
from subprocess import DEVNULL

from honcho.manager import Manager
from termcolor import cprint

import b
from utility.dockerise import defaults, platform

# NUbots RoboCup team ID
NUBOTS_TEAM_ID = 12


def register(command):
    command.help = "Build and run images for use with Webots"
    # subparsers = command.add_subparsers(help="sub-command help", dest="sub_command")

    # run_subcommand = subparsers.add_parser("run", help="Run the simulation docker image for use with Webots")
    command.add_argument("role", help="The role to run")
    command.add_argument("num_robots", type=int, help="The number of robots to run")
    command.add_argument("--single_team", action="store_true", default=False, help="Run all robots on the same team")
    command.add_argument("args", nargs="*", help="the command and any arguments that should be used for the execution")

def run(role, num_robots=1, single_team=False, ports=[], **kwargs):
    # Get the current selected image
    image = defaults.image_name("selected")

    # Honcho is used to run the docker containers in parallel
    process_manager = Manager()

    # Override honcho.manager.terminate() method, this is called by its signal handler on CTRL+C
    process_manager._killall = exec_stop

    # Add all robot run commands to process_manager
    for i in range(1, num_robots + 1):

        # Set team_id, robot_color and port_num based on the number of instances being started
        # If only a single team, or in the first half of the robots, configure for team 1
        if single_team or i <= num_robots // 2:
            team_id = NUBOTS_TEAM_ID
            webots_port = 10000 + i
            player_id = i
            hostname = f"webots1{player_id}"
        else:
            team_id = NUBOTS_TEAM_ID + 1
            webots_port = 10020 + i - num_robots // 2
            player_id = i - num_robots // 2
            hostname = f"webots2{player_id}"

        # Build a command to run the `run` command
        command = [
            "./b",
            "run",
            role,
            *kwargs["args"],
            f"--webots_port={webots_port}",
            f"--player_id={player_id}",
            f"--team_id={team_id}",
            f"--hostname={hostname}",
            f"--name=robot{i}",
        ]

        process_manager.add_process(f"Robot_{i}", " ".join(command))

    # Start the containers
    process_manager.loop()

    sys.exit(process_manager.returncode)


# Signal handler function to kill containers on CTRL+C
def exec_stop():
    # Get all container ID's for containers running this game
    container_ids = (
        subprocess.check_output(
            [
                "docker",
                "ps",
                "-a",
                "-q",
                "--filter",
                f"name=robot",
                "--format={{.ID}}",
            ]
        )
        .strip()
        .decode("ascii")
        .split()
    )

    cprint(
        f"Stopping ALL containers...",
        color="red",
        attrs=["bold"],
    )
    exit_code = subprocess.run(
        ["docker", "container", "rm", "-f"] + container_ids, stderr=DEVNULL, stdout=DEVNULL
    ).returncode

    sys.exit(exit_code)
