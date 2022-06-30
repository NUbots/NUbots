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

    # Look through the various tools to see if we can find one that matches our arguments
    # If we do we don't need to load all the tools and can just trigger this one directly
    # This saves importing things we don't need
    for path in [user_tools_path, nuclear_tools_path]:
        for dirpath, dnames, fnames in os.walk(path):
            for f in fnames:
                if f != "__init__.py" and f.endswith(".py"):

                    # Check if this is the tool for the job
                    components = os.path.relpath(os.path.join(dirpath, f[:-3]), path).split(os.sep)
                    if sys.argv[1 : len(components) + 1] == components:

                        # Load the module
                        module = pkgutil.find_loader(".".join(components)).load_module()
                        if hasattr(module, "register") and hasattr(module, "run"):

                            # Build up the base subcommands to this point
                            subcommand = subcommands
                            for c in components[:-1]:
                                subcommand = subcommand.add_parser(c).add_subparsers(
                                    dest="{}_command".format(c),
                                    help="Commands related to working with {} functionality".format(c),
                                )

                            module.register(subcommand.add_parser(components[-1]))
                            # Try to provide completion
                            try:
                                import argcomplete

                                argcomplete.autocomplete(command)
                            except ImportError:
                                pass
                            module.run(**vars(command.parse_args()))

                            # We're done, exit
                            exit(0)

    # If we reach this point, we couldn't find a tool to use.
    # In this case we need to look through all the tools so we can register them all.
    # This will provide a complete help for the function call so the user can try again
    tools = {}
    for importer, modname, ispkg in pkgutil.walk_packages([user_tools_path, nuclear_tools_path]):
        # Tools aren't in packages
        if not ispkg:

            # Load the modules and check it's a tool
            components = modname.split(".")
            try:
                module = pkgutil.find_loader(modname).load_module()
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

                    module.register(subcommand.add_parser(components[-1]))
            except ModuleNotFoundError as e:
                print("Could not load the tool '{}': {}".format(modname.replace(".", " "), e))
            except BaseException as e:
                pass

    # Try to provide completion
    try:
        import argcomplete

        argcomplete.autocomplete(command)
    except ImportError:
        pass
    # Given what we know, this will fail here and give the user some help
    command.parse_args()
