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
import re

import yaml
from termcolor import cprint

import b
from utility.dockerise import run_on_docker
from utility.roles import all_role_names
from utility.shell import WrapPty


@run_on_docker
def register(command):
    # Install help
    command.description = "Run a built binary within the local docker container"

    command.add_argument(
        "--gdb", dest="use_gdb", action="store_true", default=False, help="Run the specified program using gdb"
    )
    command.add_argument("--trace", action="store_true", default=False, help="Generate a trace file for the program")
    command.add_argument("--trace-output", default="recordings/trace.pftrace", help="The output file for the trace")
    command.add_argument("--webots_port", type=int, default=None, help="The port to use for webots")
    command.add_argument("--player_id", type=int, default=None, help="The player id to use for the player")
    command.add_argument("--team_id", type=int, default=None, help="The team id to use for the player")
    command.add_argument("args", nargs="+", help="the command and any arguments that should be used for the execution")

@run_on_docker
def run(args, use_gdb, trace, trace_output, webots_port, player_id, team_id, **kwargs):
    # Check to see if ASan was enabled
    use_asan = b.cmake_cache["USE_ASAN"] == "ON"

    # Change into the build directory
    os.chdir(os.path.join(b.project_dir, "..", "build"))

    # Add 'bin/` to the command (first argument)
    args[0] = os.path.join("bin", args[0])

    # Get current environment
    env = os.environ

    if trace:
        env["NUCLEAR_TRACE_FILE"] = trace_output

    # Make sure default log path exists
    if use_asan:
        log_path = os.path.join(os.sep, "home", "nubots", "NUbots", "log")
        os.makedirs(os.path.dirname(log_path), exist_ok=True)

    # Add necessary ASAN environment variables
    if use_asan:
        cprint("WARN: ASan is enabled. Set USE_ASAN to OFF and rebuild to disable.", "red", attrs=["bold"])

        # Append log_path option if other options have been set
        if "ASAN_OPTIONS" in env:
            # Only append log_path if it hasn't already been set
            if "log_path" not in env["ASAN_OPTIONS"]:
                env["ASAN_OPTIONS"] = f"{env['ASAN_OPTIONS']}:log_path={os.path.join(log_path, 'asan.log')}"
            # Check for shadow gap flag (needed to let cuda operations run correctly)
            if "protect_shadow_gap" not in env["ASAN_OPTIONS"]:
                env["ASAN_OPTIONS"] = f"{env['ASAN_OPTIONS']}:protect_shadow_gap=0"
        else:
            env.update({"ASAN_OPTIONS": f"log_path={os.path.join(log_path, 'asan.log')}:protect_shadow_gap=0"})

        # Find the asan log path and make sure the folder exists
        log_path = re.search("log_path=([^:]+):?", env["ASAN_OPTIONS"]).group(1)
        os.makedirs(os.path.dirname(log_path), exist_ok=True)

    # Start role with GDB
    if use_gdb:
        # Prevent guile from running auto-compilation (since they all fail and we don't need them)
        if "GUILE_AUTO_COMPILE" not in env:
            env["GUILE_AUTO_COMPILE"] = "0"

        # Setup the gdb command
        cmd = ["gdb"]

        # Disable the use of debuginfod
        # Our package versions are out of date with the debuginfod servers so trying to query for any debug symbols
        # will just waste time
        cmd.extend(["-ex", "set debuginfod enabled off"])

        # Add breakpoint to stop asan before it reports an error
        if use_asan:
            cmd.extend(["-ex", "set breakpoint pending on"])
            cmd.extend(["-ex", "br __asan::ReportGenericError"])

        # Start the role
        cmd.extend(["-ex", "r", "--args"])
    else:
        cmd = []

    # Update the config files if requested to support running multiple robots
    update_configs(webots_port, player_id, team_id)

    # Run the command
    pty = WrapPty()
    exit(pty.spawn(cmd + args, env))

# Update config
def update_config(yaml_file, key, value):
    # Try to open the file, if it fails don't do anything
    try:
        with open(yaml_file, "r") as file:
            config = yaml.load(file, Loader=yaml.FullLoader)
    except FileNotFoundError:
        print(f"config file not found: {yaml_file}, module not built.")
        return

    # Update the config
    config[key] = value

    # Write the updated config back to the file
    with open(yaml_file, "w") as file:
        yaml.dump(config, file)

def update_configs(webots_port, player_id, team_id):
    # Change into the config directory
    os.chdir("./config")

    # If the webots port is given, set it in Webots.yaml
    if webots_port is not None:
        update_config("Webots.yaml", "port", webots_port)

    # If the player id is given, set it in GlobalConfig and GameController
    if player_id is not None:
        update_config("webots/GlobalConfig.yaml", "player_id", player_id)
        update_config("GameController.yaml", "player_id", player_id)

    # Set `team_id` if it is provided
    if team_id is not None:
        update_config("webots/GlobalConfig.yaml", "team_id", team_id)

    # Change back to the build directory
    os.chdir("./..")
