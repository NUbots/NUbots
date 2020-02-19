#!/usr/bin/env python3
import argparse
import os
import pkgutil
import re
import sys

# Don't make .pyc files
sys.dont_write_bytecode = True

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
    dirs = ["build", os.path.join(os.pardir, "build")]
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

if __name__ == "__main__":

    # Add our builtin tools to the path and user tools
    sys.path.append(nuclear_tools_path)
    sys.path.append(user_tools_path)

    # Root parser information
    command = argparse.ArgumentParser(
        description="This script is an optional helper script for performing common tasks for working with the NUClear roles system."
    )
    subcommands = command.add_subparsers(
        dest="command", help="The command to run from the script. See each help for more information."
    )
    subcommands.required = True

    # Get all of the packages that are in the build tools
    modules = list(pkgutil.iter_modules(path=[nuclear_tools_path, user_tools_path]))

    # First we try to see if sys.argv[1] gives us all the information we need
    # If it does we only need to load that module directly
    # Otherwise we load every module so we can build a help for possible tools
    target_modules = [] if len(sys.argv) < 2 else [m for m in modules if not m[2] and m[1] == sys.argv[1]]
    modules = target_modules if len(target_modules) > 0 else modules

    # Our tools dictionary
    tools = {}

    # Loop through all the modules we have to set them up in the parser
    for loader, module_name, ispkg in modules:

        # Skip any packages (folders) as these are used to store useful things that are not tools
        if not ispkg:

            # Get our module, class name and registration function
            if len(modules) == 1:
                module = loader.find_module(module_name).load_module(module_name)
                tool = getattr(module, "run")
                register = getattr(module, "register")
            else:
                try:
                    module = loader.find_module(module_name).load_module(module_name)
                    tool = getattr(module, "run")
                    register = getattr(module, "register")
                except BaseException as e:
                    # Capture the exception in a variable
                    ex = e

                    # Swallow arguments for failed commands
                    register = lambda command: command.add_argument("_", nargs="*")
                    tool = lambda **kwargs: print("Cannot run this command due to the following error\n", ex)

            # Let the tool register it's arguments
            register(subcommands.add_parser(module_name))

            # Associate our module_name with this tool
            tools[module_name] = tool

    # Parse our arguments
    args = command.parse_args()

    # Pass to our tool
    tools[args.command](**vars(args))
