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

import asyncio
import os
import re
import signal
import subprocess
import sys
import termios
from subprocess import DEVNULL

from colorama import Back, Fore, Style, init
from termcolor import cprint

# NUbots RoboCup team ID
NUBOTS_TEAM_ID = 12

# Lock for printing
print_lock = asyncio.Lock()

# Initialize colorama for Windows support
init(autoreset=True)
# Robot colours based on the team and player id (up to 10 players per team)
ROBOT_COLOURS = [
    Fore.RED, Fore.GREEN, Fore.YELLOW, Fore.BLUE, Fore.MAGENTA, Fore.CYAN, Fore.WHITE,
    Fore.LIGHTBLACK_EX, Fore.LIGHTWHITE_EX, Fore.LIGHTYELLOW_EX
]

original_termios_settings = None

def register(command):
    command.help = "Build and run images for use with Webots"
    command.add_argument("role", help="The role to run")
    command.add_argument("num_robots", type=int, help="The number of robots to run")
    command.add_argument("--single_team", action="store_true", default=False, help="Run all robots on the same team")
    command.add_argument("args", nargs="*", help="the command and any arguments that should be used for the execution")
async def safe_print(line):
    async with print_lock:
        print(line)

async def run_process(name, command, colour):
    process = await asyncio.create_subprocess_exec(
        *command,
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.PIPE
    )

    await safe_print(f"{colour}[{name}] started with PID {process.pid}{Style.RESET_ALL}")

    async def read_stream(stream, stream_name):
        while True:
            line = await stream.readline()
            if not line:
                break
            line = line.decode().rstrip()
            # Build the entire line *before* printing
            log_line = f"{colour}[{name} {stream_name}] {line}{Style.RESET_ALL}"
            await safe_print(log_line)

    await asyncio.gather(
        read_stream(process.stdout, "stdout"),
        read_stream(process.stderr, "stderr")
    )

    return_code = await process.wait()
    await safe_print(f"{colour}[{name}] exited with return code {return_code}{Style.RESET_ALL}")
async def async_run(role, num_robots=1, single_team=False, ports=[], **kwargs):
    commands = []

    for i in range(1, num_robots + 1):
        robot_colour = ROBOT_COLOURS[i % len(ROBOT_COLOURS)]  # Assign color based on robot number

        command = [
            "./b",
            "run",
            role,
            *kwargs["args"],
            "--webots_port",
            str(10000 + i),
            "--player_id",
            str(i),
            "--team_id",
            str(NUBOTS_TEAM_ID),
        ]
        commands.append((f"Robot_{NUBOTS_TEAM_ID}_{i}", command, robot_colour))

        if single_team:
            continue

        command = [
            "./b",
            "run",
            role,
            *kwargs["args"],
            "--webots_port",
            str(10020 + i),
            "--player_id",
            str(i),
            "--team_id",
            str(NUBOTS_TEAM_ID + 1),
        ]
        commands.append((f"Robot_{NUBOTS_TEAM_ID+1}_{i}", command, robot_colour))

    await asyncio.gather(*(run_process(name, cmd, colour) for name, cmd, colour in commands))

def run(**kwargs):
    global original_termios_settings
    try:
        fd = sys.stdin.fileno()
        if os.isatty(fd):
            original_termios_settings = termios.tcgetattr(fd)
    except Exception:
        original_termios_settings = None

    try:
        asyncio.run(async_run(**kwargs))
    except KeyboardInterrupt:
        exec_stop()
    finally:
        # Make sure we ALWAYS reset terminal no matter how we exit
        force_terminal_reset()

def force_terminal_reset():
    try:
        if original_termios_settings is not None:
            fd = sys.stdin.fileno()
            if os.isatty(fd):
                termios.tcsetattr(fd, termios.TCSADRAIN, original_termios_settings)
    except Exception:
        pass

def exec_stop():
    try:
        container_ids = (
            subprocess.check_output(
                [
                    "docker",
                    "ps",
                    "-a",
                    "-q",
                    "--filter",
                    "name=webots",
                    "--format={{.ID}}",
                ]
            )
            .strip()
            .decode("ascii")
            .split()
        )

        matching_containers = []
        for container_id in container_ids:
            hostname = (
                subprocess.check_output(
                    [
                        "docker",
                        "inspect",
                        "-f",
                        "{{.Config.Hostname}}",
                        container_id,
                    ]
                )
                .strip()
                .decode("ascii")
            )
            if re.match(r"webots\d+", hostname):
                matching_containers.append(container_id)

        if matching_containers:
            cprint(f"Stopping ALL containers...", color="red", attrs=["bold"])
            subprocess.run(
                ["docker", "container", "rm", "-f"] + matching_containers,
                stderr=DEVNULL,
                stdout=DEVNULL
            )
    except subprocess.CalledProcessError:
        pass

    # Try to flush all output
    try:
        sys.stdout.flush()
        sys.stderr.flush()
    except Exception:
        pass

    # **Force reset terminal properly**
    force_terminal_reset()

    sys.exit(0)
