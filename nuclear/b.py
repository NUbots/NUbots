#!/usr/bin/env python3
import sys
import os
import argparse
import pkgutil
import re

# Don't make .pyc files
sys.dont_write_bytecode = True

# Go and get all the relevant directories and variables from cmake
nuclear_dir = os.path.dirname(os.path.realpath(__file__))
project_dir = os.path.dirname(nuclear_dir)

# Get the tools directories to find b modules
nuclear_tools_path = os.path.join(nuclear_dir, 'tools')
user_tools_path    = os.path.join(project_dir, 'tools')

# Build our cmake cache
cmake_cache = {}

# Try to find our cmake cache file in the pwd
if os.path.isfile('CMakeCache.txt'):
    with open('CMakeCache.txt', 'r') as f:
        cmake_cache_text = f.readlines()

# Look in a build directory
elif os.path.isfile(os.path.join(project_dir, 'build', 'CMakeCache.txt')):
    with open(os.path.join(project_dir, 'build', 'CMakeCache.txt'), 'r') as f:
        cmake_cache_text = f.readlines()

# Go and process our lines in our cmake file
for l in cmake_cache_text:

    # Remove whitespace at the ends and start
    l = l.strip()

    # Remove lines that are comments
    if len(l) > 0 and not l.startswith('//') and not l.startswith('#'):
        # Extract our variable name from our values
        g = re.match(r'([a-zA-Z_$][a-zA-Z_.$0-9-]*):(\w+)=(.*)', l).groups()

        # Store our value and split it into a list if it is a list
        cmake_cache[g[0]] = g[2] if ';' not in g[2].strip(';') else g[2].strip(';').split(';');

binary_dir = cmake_cache[cmake_cache["CMAKE_PROJECT_NAME"] + '_BINARY_DIR']
source_dir = cmake_cache[cmake_cache["CMAKE_PROJECT_NAME"] + '_SOURCE_DIR']

if __name__ == "__main__":

    # Print some information for the user
    print("b script for", cmake_cache["CMAKE_PROJECT_NAME"])
    print("\tSource:", cmake_cache[cmake_cache["CMAKE_PROJECT_NAME"] + '_SOURCE_DIR'])
    print("\tBinary:", cmake_cache[cmake_cache["CMAKE_PROJECT_NAME"] + '_BINARY_DIR'])
    print()

    # Add our builtin tools to the path and user tools
    sys.path.append(nuclear_tools_path)
    sys.path.append(user_tools_path)

    if __name__ == '__main__':

        # Root parser information
        command = argparse.ArgumentParser(description='This script is an optional helper script for performing common tasks for working with the NUClear roles system.')
        subcommands = command.add_subparsers(dest='command')
        subcommands.help = "The command to run from the script. See each help for more information."

        # Get all of the packages that are in the build tools
        modules = pkgutil.iter_modules(path=[nuclear_tools_path, user_tools_path])

        # Our tools dictionary
        tools = {}

        # Loop through all the modules we have to set them up in the parser
        for loader, module_name, ispkg in modules:

            # Get our module, class name and registration function
            module = loader.find_module(module_name).load_module(module_name)
            tool = getattr(module, 'run')
            register = getattr(module, 'register')

            # Let the tool register it's arguments
            register(subcommands.add_parser(module_name))

            # Associate our module_name with this tool
            tools[module_name] = tool

        # Parse our arguments
        args = command.parse_args()

        # Pass to our tool
        tools[args.command](**vars(args))
