#!/usr/bin/env -S uv run --script
#
# MIT License
#
# Copyright (c) 2013 NUbots
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
import argparse
import asyncio
import importlib.util
import os
import re
import sys

# Don't make .pyc files
sys.dont_write_bytecode = True


# Populate this with dependencies for the script as needed
class Dependencies:
    def __init__(self):
        self.dependencies = set()

    def register(self, *new_deps):
        """Register new dependencies that will be used when re-executing the script"""
        self.dependencies.update(new_deps)

    def get(self):
        """Get the current set of dependencies"""
        return self.dependencies


dependencies = Dependencies()

# Ensure this module is in sys.modules
sys.modules["b"] = sys.modules[__name__]


def reexecute_with_dependencies():
    """Re-execute the script with additional dependencies if needed"""
    current_deps = (
        set() if "B_EXTRA_DEPS" not in os.environ else set([d for d in os.environ["B_EXTRA_DEPS"].split(",") if d])
    )

    # Only re-execute if we have new dependencies that aren't already included
    if not dependencies.get().issubset(current_deps):
        # Combine the dependencies
        deps_list = list(sorted(current_deps.union(dependencies.get())))
        os.environ["B_EXTRA_DEPS"] = ",".join(deps_list)

        # Build the uv run command with dependencies
        cmd = [
            "uv",
            "run",
            *[arg for dep in deps_list for arg in ["--with", dep]],
            __file__,
            *sys.argv[1:],
        ]

        # Re-execute through uv run directly
        os.execvpe("uv", cmd, os.environ)


# Go and get all the relevant directories and variables from cmake
nuclear_dir = os.path.dirname(os.path.realpath(__file__))
project_dir = os.path.dirname(nuclear_dir)

# Get the tools directories to find b modules
nuclear_tools_path = os.path.join(nuclear_dir, "tools")
user_tools_path = os.path.join(project_dir, "tools")

# Build our cmake cache
cmake_cache = {}

# Try to find our cmake cache file in the pwd
if os.path.isfile("CMakeCache.txt"):
    with open("CMakeCache.txt", "r") as f:
        cmake_cache_text = f.readlines()


# Look for a build directory
else:
    dirs = [os.path.join(os.pardir, "build"), "build"]
    try:
        dirs.extend([os.path.join("build", f) for f in os.listdir("build")])
    except FileNotFoundError:
        pass

    for d in dirs:
        if os.path.isfile(os.path.join(project_dir, d, "CMakeCache.txt")):
            with open(os.path.join(project_dir, d, "CMakeCache.txt"), "r") as f:
                cmake_cache_text = f.readlines()
            break

    # If we still didn't find anything
    try:
        cmake_cache_text
    except NameError:
        cmake_cache_text = []

# Go and process our lines in our cmake file
for l in cmake_cache_text:

    # Remove whitespace at the ends and start
    l = l.strip()

    # Remove lines that are comments
    if len(l) > 0 and not l.startswith("//") and not l.startswith("#"):
        # Extract our variable name from our values
        g = re.match(r"([a-zA-Z_$][a-zA-Z_.$0-9-]*):(\w+)=(.*)", l).groups()

        # Store our value and split it into a list if it is a list
        cmake_cache[g[0]] = g[2] if ";" not in g[2].strip(";") else g[2].strip(";").split(";")

# Try to find our source and binary directories
try:
    binary_dir = cmake_cache[cmake_cache["CMAKE_PROJECT_NAME"] + "_BINARY_DIR"]
except KeyError:
    binary_dir = None

try:
    source_dir = cmake_cache[cmake_cache["CMAKE_PROJECT_NAME"] + "_SOURCE_DIR"]
except:
    source_dir = project_dir


async def main():

    # Prepend nuclear and user tools to the path, so we prefer our packages
    sys.path.insert(0, nuclear_tools_path)
    sys.path.insert(0, user_tools_path)

    # Root parser information
    command = argparse.ArgumentParser(
        description="This script is an optional helper script for performing common tasks for working with the NUClear roles system."
    )
    subcommands = command.add_subparsers(
        dest="command", help="The command to run from the script. See each help for more information."
    )
    subcommands.required = True

    # Look through our tools directories and find all the files and folders that could be a command
    candidates = []
    for path in [user_tools_path, nuclear_tools_path]:
        for dirpath, dnames, fnames in os.walk(path):

            # Get all the possible commands they might want to run based on folders and python files
            candidates.extend(
                [
                    os.path.relpath(os.path.join(dirpath, os.path.splitext(f)[0]), path).split(os.sep)
                    for f in fnames
                    if f != "__init__.py" and os.path.splitext(f)[1] == ".py"
                ]
            )
            candidates.extend(
                [
                    os.path.relpath(os.path.join(dirpath, d), path).split(os.sep)
                    for d in dnames
                    if os.path.isfile(os.path.join(dirpath, d, "__init__.py"))
                ]
            )

    # See if we can find a command that matches what we want to do and sort so the longest match is first
    useable = [c for c in candidates if sys.argv[1 : len(c) + 1] == c]
    useable.sort(key=lambda x: len(x), reverse=True)

    # If we can't find a match, try subsequence matching the input arguments to find a command
    if not useable:
        # Matching first letter, and then the rest of the letters in order
        substring_candidates = [
            c
            for c in candidates
            if all(
                r.fullmatch(c)
                for r, c in zip(
                    (re.compile(".*".join(map(re.escape, a)) + ".*") for a in sys.argv[1 : len(c) + 1]),
                    c,
                )
            )
        ]
        # Only allow if the fuzzy match is unique
        if len(substring_candidates) == 1:
            useable = substring_candidates
            # Replace the fuzzy arguments with the matched command
            cmd = useable[0]
            sys.argv[1 : len(cmd) + 1] = cmd

    for components in useable:
            module_name = ".".join(components)
            # If this is a submodule, ensure the parent package is loaded first
            if len(components) > 1:
                parent_module_name = ".".join(components[:-1])
                if parent_spec := importlib.util.find_spec(parent_module_name):
                    parent_module = importlib.util.module_from_spec(parent_spec)
                    sys.modules[parent_module_name] = parent_module
                    parent_spec.loader.exec_module(parent_module)

            # Re-run with updated dependencies if we need to (before loading the module)
            reexecute_with_dependencies()

            if spec := importlib.util.find_spec(module_name):
                module = importlib.util.module_from_spec(spec)
                sys.modules[module_name] = module
                spec.loader.exec_module(module)
                if hasattr(module, "register") and hasattr(module, "run"):
                    # Build up the base subcommands to this point
                    subcommand = subcommands
                    for c in components[:-1]:
                        subcommand = subcommand.add_parser(c).add_subparsers(
                            dest="{}_command".format(c),
                            help="Commands related to working with {} functionality".format(c),
                        )
                    subcommand.required = True

                    if asyncio.iscoroutinefunction(module.register):
                        await module.register(subcommand.add_parser(components[-1]))
                    else:
                        module.register(subcommand.add_parser(components[-1]))

                    # Run the module
                    if asyncio.iscoroutinefunction(module.run):
                        await module.run(**vars(command.parse_args()))
                    else:
                        module.run(**vars(command.parse_args()))

                    # We're done, exit
                    exit(0)

    # If we reach this point, we couldn't find a tool to use.
    # In this case we need to look through all the tools so we can register them all.
    # This will provide a complete help for the function call so the user can try again
    tools = {}
    for components in candidates:
        module_name = ".".join(components)
        if spec := importlib.util.find_spec(module_name):
            module = importlib.util.module_from_spec(spec)
            sys.modules[module_name] = module
            spec.loader.exec_module(module)

            if hasattr(module, "register") and hasattr(module, "run"):
                subcommand = subcommands
                tool = tools
                for c in components[:-1]:
                    if c in tool:
                        tool, subcommand = tool[c]
                    else:
                        subcommand = subcommand.add_parser(c).add_subparsers(
                            dest="{}_command".format(c),
                            help="Commands related to working with {} functionality".format(c),
                        )
                        subcommand.required = True
                        tool[c] = ({}, subcommand)
                        tool = tool[c][0]

                if asyncio.iscoroutinefunction(module.register):
                    await module.register(subcommand.add_parser(components[-1]))
                else:
                    module.register(subcommand.add_parser(components[-1]))

    # Given what we know, this will fail here and give the user some help
    command.parse_args()


if __name__ == "__main__":
    asyncio.run(main())
