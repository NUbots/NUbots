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

import os
import shutil
import subprocess
import sys
from subprocess import DEVNULL

from termcolor import cprint


def ensure_docker():
    """Exit with a helpful message if the Docker CLI is missing or the daemon is unreachable."""
    if shutil.which("docker") is None:
        cprint("Docker is not installed (could not find `docker` on PATH).", "red", attrs=["bold"])
        exit(1)

    if subprocess.run(["docker", "info"], stdout=DEVNULL, stderr=DEVNULL).returncode == 0:
        return

    cprint("Docker is not currently running.", "red", attrs=["bold"])
    if sys.platform == "darwin":
        cprint("Ensure Docker Desktop is currently running, and Rosetta is turned on.", "red", attrs=["bold"])
    elif "microsoft" in os.uname().release.lower():
        cprint(
            "Ensure Docker Desktop is currently running, and that WSL2 integration is enabled.", "red", attrs=["bold"]
        )
    else:
        cprint("Ensure the docker daemon is running (systemctl enable --now docker).", "red", attrs=["bold"])
    exit(1)
