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

import asyncio
import os
import re
import subprocess
import sys
import termios
from contextlib import contextmanager
from subprocess import DEVNULL

from colorama import Fore, Style, init
from termcolor import cprint

# NUbots RoboCup team ID
NUBOTS_TEAM_ID = 12

# Lock for printing
print_lock = asyncio.Lock()


def register(command):
    command.help = "Build and run images for use with Webots"
    command.add_argument("role", help="The role to run")
    command.add_argument("num_robots", type=int, help="The number of robots to run")
    command.add_argument("--single_team", action="store_true", default=False, help="Run all robots on the same team")
    command.add_argument("args", nargs="*", help="the command and any arguments that should be used for the execution")


# Global to store the original terminal settings
original_termios_settings = None


# Context manager is used to reset the terminal settings on exit
@contextmanager
def terminal_reset():
    global original_termios_settings
    try:
        fd = sys.stdin.fileno()
        if os.isatty(fd):
            original_termios_settings = termios.tcgetattr(fd)
        yield
    finally:
        if original_termios_settings is not None:
            termios.tcsetattr(fd, termios.TCSADRAIN, original_termios_settings)


# This function is used to print messages in a thread-safe manner
async def safe_print(line):
    async with print_lock:
        sys.stdout.write("\r" + line + "\n")
        sys.stdout.flush()


# This function runs a process and captures its output
async def run_process(name, command, colour):
    process = await asyncio.create_subprocess_exec(
        *command, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE
    )

    await safe_print(f"{colour}[{name}] started with PID {process.pid}{Style.RESET_ALL}")

    async def read_stream(stream, stream_name):
        while True:
            line = await stream.readline()
            if not line:
                break
            line = line.decode().rstrip()
            # Build the entire line before printing
            log_line = f"{colour}[{name} {stream_name}] {line}{Style.RESET_ALL}"
            await safe_print(log_line)

    await asyncio.gather(read_stream(process.stdout, "stdout"), read_stream(process.stderr, "stderr"))

    return_code = await process.wait()
    await safe_print(f"{colour}[{name}] exited with return code {return_code}{Style.RESET_ALL}")


# Runs the requested number of robots with the `./b run` command
async def run(role, num_robots=1, single_team=False, **kwargs):
    with terminal_reset():
        try:
            # Initialise colorama for Windows support
            init(autoreset=True)
            # Robot colours based on the team and player id (up to 10 players per team)
            ROBOT_COLOURS = [
                Fore.RED,
                Fore.GREEN,
                Fore.YELLOW,
                Fore.BLUE,
                Fore.MAGENTA,
                Fore.CYAN,
                Fore.WHITE,
                Fore.LIGHTBLACK_EX,
                Fore.LIGHTWHITE_EX,
                Fore.LIGHTYELLOW_EX,
            ]

            def build_command(role, args, webots_port, player_id, team_id):
                return [
                    "./b",
                    "run",
                    role,
                    *args,
                    "--webots_port",
                    str(webots_port),
                    "--player_id",
                    str(player_id),
                    "--team_id",
                    str(team_id),
                ]

            commands = []

            # Loop over each robot
            for i in range(1, num_robots + 1):
                # Assign color based on robot number
                robot_colour = ROBOT_COLOURS[i % len(ROBOT_COLOURS)]

                # Add command for the first team
                commands.append(
                    (
                        f"Robot_{NUBOTS_TEAM_ID}_{i}",
                        build_command(role, kwargs["args"], 10000 + i, i, NUBOTS_TEAM_ID),
                        robot_colour,
                    )
                )

                # Add command for the second team if not single_team
                if not single_team:
                    commands.append(
                        (
                            f"Robot_{NUBOTS_TEAM_ID + 1}_{i}",
                            build_command(role, kwargs["args"], 10020 + i, i, NUBOTS_TEAM_ID + 1),
                            robot_colour,
                        )
                    )

            # Run the processes concurrently
            await asyncio.gather(*(run_process(name, cmd, colour) for name, cmd, colour in commands))

        except KeyboardInterrupt:
            exec_stop()


# Clean up on exit
def exec_stop():
    # Stop the containers, otherwise they will continue to run
    try:
        container_ids = (
            subprocess.check_output(["docker", "ps", "-a", "-q", "--filter", "name=webots", "--format={{.ID}}"])
            .decode("ascii")
            .strip()
            .split()
        )
        # This regex matches the container name set in the dockerise/run script
        matching_containers = [
            container_id
            for container_id in container_ids
            if re.match(
                r"webots\d+",
                subprocess.check_output(["docker", "inspect", "-f", "{{.Config.Hostname}}", container_id])
                .decode("ascii")
                .strip(),
            )
        ]

        if matching_containers:
            cprint("Stopping ALL containers...", color="red", attrs=["bold"])
            subprocess.run(["docker", "container", "rm", "-f"] + matching_containers, stderr=DEVNULL, stdout=DEVNULL)
    except subprocess.CalledProcessError:
        pass
    finally:
        sys.exit(0)
