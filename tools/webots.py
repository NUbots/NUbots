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
import signal
import subprocess
import sys
import textwrap
from pathlib import Path
from subprocess import DEVNULL

from honcho.manager import Manager, Printer
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
    procfile_lines = []

    # Add all robot run commands to process_manager
    for i in range(1, num_robots + 1):

        # Command for first team player i
        command = [
            "./b",
            "run",
            role,
            *kwargs["args"],
            "--webots_port", str(10000 + i),
            "--player_id", str(i),
            "--team_id", str(NUBOTS_TEAM_ID),
        ]
        procfile_lines.append(f"Robot_{i}: {' '.join(command)}")

        # # Skip second team if single_team is True
        # if single_team:
        #     continue

        # # Build a command to run the `run` command
        # command = [
        #     "./b",
        #     "run",
        #     role,
        #     *kwargs["args"],
        #     "--webots_port", str(10020 + i),
        #     "--player_id", str(i),
        #     "--team_id", str(NUBOTS_TEAM_ID + 1),
        # ]

        # process_manager.add_process(f"Robot_{i*2}", " ".join(command))

    # Write to temporary Procfile
    procfile_path = Path(".overmind_procfile")
    procfile_path.write_text("\n".join(procfile_lines))

    def handle_sigint(signum, frame):
        print("\n[Python] Received Ctrl+C, stopping containers...")
        exec_stop()

    signal.signal(signal.SIGINT, handle_sigint)

    try:
        subprocess.run(["overmind", "start", "-f", str(procfile_path)], check=True)
    except subprocess.CalledProcessError as e:
        sys.exit(e.returncode)


# Signal handler function to kill containers on CTRL+C
def exec_stop():
    # Get all container IDs for containers with "webots" in their hostname
    container_ids = (
        subprocess.check_output(
            [
                "docker",
                "ps",
                "-a",
                "-q",
                "--filter",
                "name=webots",  # Match containers with "webots" in their name
                "--format={{.ID}}",
            ]
        )
        .strip()
        .decode("ascii")
        .split()
    )

    # Filter containers with hostnames matching "webots<number>"
    matching_containers = []
    for container_id in container_ids:
        hostname = subprocess.check_output(
            [
                "docker",
                "inspect",
                "-f",
                "{{.Config.Hostname}}",
                container_id,
            ]
        ).strip().decode("ascii")
        if re.match(r"webots\d+", hostname):  # Match "webots" followed by a number
            matching_containers.append(container_id)

    cprint(
        f"Stopping ALL containers...",
        color="red",
        attrs=["bold"],
    )
    exit_code = subprocess.run(
        ["docker", "container", "rm", "-f"] + matching_containers, stderr=DEVNULL, stdout=DEVNULL
    ).returncode

    sys.exit(exit_code)
