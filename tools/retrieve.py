#!/usr/bin/env python3
#
# MIT License
#
# Copyright (c) 2026 NUbots
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

import glob
import os
import subprocess

from termcolor import cprint

import b
from utility.dockerise import run_on_docker

# Preset names that expand to (remote path, default local directory)
# Trailing slash on the remote path makes rsync copy the directory contents
PRESETS = {
    "configs": ("/home/nubots/config/", "configs", True), # append timestamp is True for configs
    "logs": ("/home/nubots/recordings/", "recordings", False),
}


@run_on_docker
def register(command):
    command.description = "Retrieve files from the target system to the local system"

    command.add_argument("host", help="The host or directory to retrieve the files from")

    command.add_argument(
        "target",
        help=f"The target directory/files to get, or a preset name: {', '.join(PRESETS)}",
    )

    command.add_argument(
        "local",
        nargs="?",
        default=None,
        help="The local directory to retrieve the files to (defaults per preset)",
    )

    command.add_argument("--user", "-u", help="The user to retrieve the files with")

    command.add_argument("--append-timestamp", "-t", action="store_true", help="Append a timestamp to the local name")

@run_on_docker
def run(host, target, local, user=None, **kwargs):
    # Replace hostname with its IP address if the hostname is already known
    num_robots = 4
    host = {
        "{}{}".format(prefix, num): "10.1.1.{}".format(num)
        for num in range(1, num_robots + 1)
        for prefix in ("nugus", "n", "i", "igus")
    }.get(host, host)

    # Expand preset names into their remote path and default local directory
    if target in PRESETS:
        remote_path, default_local, append_timestamp = PRESETS[target]
        target = remote_path
        if local is None:
            local = default_local
    elif local is None:
        raise SystemExit("A local directory is required when not using a preset")

    if not os.path.exists(local):
        os.makedirs(local)

    cprint(f"Retrieving files from {host}:{target} to {local}", "green")

    if user is None:
        user = "nubots"

    if append_timestamp: # make the timestamped local directory if the preset requires it
        import datetime
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S") # this is UTC.
        local = os.path.join(local, f"{timestamp}")
        os.makedirs(local, exist_ok=True)

    subprocess.run([
        "rsync",
        "-aP", # partial progression, archive mode
        "-e",  # specify the remote shell to use
        "ssh",
        f"{user}@{host}:{target}",
        f"{local}",
    ], check=True)
